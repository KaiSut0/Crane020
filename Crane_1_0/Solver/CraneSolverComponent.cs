using System;
using System.Collections.Generic;
using System.Drawing;
using System.Windows.Forms;

using Grasshopper.Kernel;
using Grasshopper.Kernel.Attributes;
using Grasshopper.GUI;
using Grasshopper.GUI.Canvas;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;

using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace Crane
{
    public class CraneSolverComponent : GH_Component
    {
        CMesh cmesh;
        Mesh mesh;
        System.Windows.Forms.Timer timer;
        int moveVertIndex;
        bool mouseDown;
        private List<double> EdgeLengthSquared;
        bool IsSimulationMode;

        bool isOn = false;

        public bool Fold = false;
        public bool Unfold = false;
        public bool Reset = false;
        public bool SolverOn = false;
        public bool RigidModeOn = false;
        public bool QuadFlatOn = false;
        public bool FoldBlockOn = false;

        bool start = true;

        Rhino.UI.MouseCallback my_mouse;

        public CraneSolverComponent()
          : base("CraneSolver", "CraneSolver",
              "Solver for simulation and design",
              "Crane", "Solver")
        {
            cmesh = null;
            mesh = null;
            timer = new System.Windows.Forms.Timer();
            timer.Interval = 30;
            timer.Tick += new EventHandler(tick);
            mouseDown = false;
            EdgeLengthSquared = new List<double>();
            IsSimulationMode = false;
        }

        public override void CreateAttributes()
        {
            m_attributes = new Attributes_Custom(this);
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "CMesh", GH_ParamAccess.item);
            pManager.AddGenericParameter("Constraints", "C", "Constraints", GH_ParamAccess.list);
            pManager.AddNumberParameter("FoldSpeed", "FoldSpeed", "Fold speed between 0 - 1", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Rigidity", "Rigidity", "a", GH_ParamAccess.item);

            //Constraintsは入力なしでも動く
            pManager[1].Optional = true;
            pManager[2].Optional = true;
            pManager[3].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "CMesh", GH_ParamAccess.item);
            pManager.AddTextParameter("Residual", "r", "residual", GH_ParamAccess.item);

        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            string debug = "";
            List<Line> edges = new List<Line>();

            //入力を格納する変数宣言、初期化
            CMesh cmeshIn = null;
            List<Constraint> Constraints = new List<Constraint>();
            bool Sim = false;
            bool On = false;
            double FoldSpeed = 0;
            int rigidity = 1;


            //入力の有無確認＆格納
            {
                if (!DA.GetData(0, ref cmeshIn)) { return; }
                DA.GetDataList(1, Constraints);
                DA.GetData(2, ref FoldSpeed);
                DA.GetData(3, ref rigidity);
            }

            FoldSpeed /= 50;
            On = SolverOn;
            Sim = RigidModeOn;

            //開始時、リセット時の処理
            if (start || Reset)
            {
                //meshを入力のものにリフレッシュ
                this.cmesh = new CMesh(cmeshIn);
                this.mesh = this.cmesh.mesh;

                //エッジ長をリフレッシュ
                UpdateEdgeLengthSquared();

                //フェイスの初期高さリフレッシュ
                UpdateFacePairBasicInfo(cmesh);
                UpdateTriangulatedFacePairBasicInfo(cmesh);
                UpdateMountainFacePairBasicInfo(cmesh);
                UpdateValleyFacePairBasicInfo(cmesh);

                start = false;
                timer.Start();

            }

            //

            //simulationモードに移行したときにエッジ長をリフレッシュ
            if (Sim)
            {
                if (!this.IsSimulationMode)
                {
                    UpdateEdgeLengthSquared();
                    UpdateFacePairBasicInfo(cmesh);
                    UpdateTriangulatedFacePairBasicInfo(cmesh);
                    UpdateMountainFacePairBasicInfo(cmesh);
                    UpdateValleyFacePairBasicInfo(cmesh);
                    IsSimulationMode = true;
                }
            }
            else
            {
                IsSimulationMode = false;
            }

            //Fold時処理
            if ((Fold & ((Sim) | (Constraints.Count != 0))) & ((cmesh.mountain_edges.Count + cmesh.valley_edges.Count) != 0))
            {
                //座標ベクトル作成
                Vec coords = new Vec(mesh);
                Vec Vmove = new Vec();

                //Compute A and b for Ax = b
                CRS J = ComputeFoldJacobian(Constraints, Sim);
                CRS JOfFold = ComputeFoldAngleJacobian();
                SparseMatrix A = ComputeFoldMotionMatrix(JOfFold, J, 10);
                DenseVector driving_force;
                if (!Unfold)
                {
                    driving_force = ComputeInitialFoldAngleVector(FoldSpeed);
                }
                else
                {
                    driving_force = ComputeInitialFoldAngleVector2(FoldSpeed);
                }
                DenseVector b = ComputeFoldMotionVector(JOfFold, driving_force);

                //Solve Ax = b
                Vmove = CGMethodForSymmetricPositiveDefinite(A, b, 1e-6, A.ColumnCount);

                //Update Mesh coordinates
                coords += -1.0 * Vmove;
                this.UpdateMesh(coords);

            }

            //On時処理
            if (On)
            {
                if (!this.isOn)
                {
                    this.timer.Start();
                    this.isOn = true;
                }
                this.isOn = true;
                //座標ベクトル作成
                Vec coords = new Vec(mesh);

                //入力ベクトル作成パート/////////////////////////////////////////////////
                List<double> move = new List<double>();

                if ((Control.ModifierKeys & Keys.Alt) == Keys.Alt)
                {
                    my_mouse = new MyMouseCallback();
                    my_mouse.Enabled = true;
                }

                else if ((Control.ModifierKeys & Keys.Alt) != Keys.Alt)
                {
                    my_mouse = new MyMouseCallbackOff();
                    my_mouse.Enabled = true;
                }

                if ((Control.MouseButtons & MouseButtons.Left) == MouseButtons.Left && (Control.ModifierKeys & Keys.Alt) == Keys.Alt)
                {
                    //Grab入力
                    var doc = Rhino.RhinoDoc.ActiveDoc;

                    var p1 = Cursor.Position;
                    var p2 = doc.Views.ActiveView.ScreenToClient(p1);
                    var l = doc.Views.ActiveView.ActiveViewport.ClientToWorld(p2);
                    var dir = doc.Views.ActiveView.ActiveViewport.CameraDirection;
                    Plane pl = new Plane(new Point3d(0, 0, 0), dir);
                    double t;
                    Rhino.Geometry.Intersect.Intersection.LinePlane(l, pl, out t);

                    if (!mouseDown)
                    {
                        //最近傍頂点インデックス
                        double champ = 999999;
                        for (int i = 0; i < mesh.Vertices.Count; i++)
                        {
                            double dtemp = l.DistanceTo(mesh.Vertices[i], false);
                            if (dtemp < champ)
                            {
                                moveVertIndex = i;
                                champ = dtemp;
                            }
                        }
                        //クリックされたときは取り合えずゼロベクトル
                        for (int i = 0; i < mesh.Vertices.Count; i++)
                        {
                            move.Add(0);
                            move.Add(0);
                            move.Add(0);
                        }
                        mouseDown = true;
                    }
                    else
                    {
                        //ドラッグされてる間は、(カーソルからのレイ)と頂点を使って入力ベクトル生成
                        Point3d mouse = l.ClosestPoint(this.mesh.Vertices[this.moveVertIndex], false);
                        Vector3d Vtemp = this.mesh.Vertices[this.moveVertIndex] - mouse;
                        for (int i = 0; i < mesh.Vertices.Count; i++)
                        {
                            if (i == this.moveVertIndex)
                            {
                                move.Add((double)Vtemp.X);
                                move.Add((double)Vtemp.Y);
                                move.Add((double)Vtemp.Z);
                            }
                            else
                            {
                                move.Add(0);
                                move.Add(0);
                                move.Add(0);
                            }
                        }
                    }
                }
                else
                {
                    //Grab入力なしの時はゼロベクトル
                    for (int i = 0; i < mesh.Vertices.Count * 3; i++)
                    {
                        move.Add(0);
                    }
                    mouseDown = false;
                }

                Vec Vmove = new Vec(move);


                //CG法を使ってMesh更新パート/////////////////////////////////////////////////

                //ヤコビアン、エラー計算
                CRS J = ComputeJacobian(Constraints, Sim);
                Vec E = ComputeError(Constraints, Sim);

                //debug += J.ToString();


                if (J.Var.Count != 0)
                {
                    //simlationモードor幾何拘束ありの処理
                    int iterationMax = Math.Min(J.C_Size, J.R_Size);
                    if (Vmove.NormSquared() == 0)
                    {
                        //入力ベクトルがゼロの時、
                        int i = 0;
                        while (i < rigidity && E.NormSquared() > 1e-10)
                        {
                            Vec cg = CGMethod(J, E, Vmove, E.NormSquared() / 100, iterationMax);

                            // Start Compute Line Search/////////////////////////////////////////////
                            double GoldenRatio = ((Math.Sqrt(5.0) - 1.0) / 2.0);

                            // Compute Upper and Lower Bound
                            Vec lb = 0 * cg;
                            Vec ub = 1 * cg;

                            Vec x1, x2; // 内分点（「１：φ」と「φ：１」に分割する点）
                            double e1, e2; // 内分点での誤差値

                            // 内分点を計算
                            x1 = (1.0 / (GoldenRatio + 1.0)) * (GoldenRatio * lb + ub);
                            UpdateMesh(coords + x1);
                            E = ComputeError(Constraints, Sim);
                            e1 = E.NormSquared();

                            x2 = (1.0 / (GoldenRatio + 1.0)) * (lb + GoldenRatio * ub);
                            UpdateMesh(coords + x2);
                            E = ComputeError(Constraints, Sim);
                            e2 = E.NormSquared();


                            for (int j = 0; j < 5; j++)
                            {
                                if (e1 < e2)
                                {
                                    ub = x2;
                                    x2 = x1;
                                    e2 = e1;
                                    x1 = (1.0 / (GoldenRatio + 1.0)) * (ub - lb) + lb;
                                    this.UpdateMesh(coords + x1);
                                    E = ComputeError(Constraints, Sim);
                                    e1 = E.NormSquared();
                                }
                                else
                                {
                                    lb = x1;
                                    x1 = x2;
                                    e1 = e2;
                                    x2 = (1.0 / GoldenRatio) * (ub - lb) + lb;
                                    this.UpdateMesh(coords + x2);
                                    E = ComputeError(Constraints, Sim);
                                    e2 = E.NormSquared();
                                }
                            }
                            Vec ls = (1.0 / 2.0) * (lb + ub);
                            coords += ls;
                            // End Compute Line Serch//////////////////////////////////////////////////////////////////

                            UpdateMesh(coords);
                            J = ComputeJacobian(Constraints, Sim);
                            E = ComputeError(Constraints, Sim);
                            Vmove = 0 * Vmove;
                            debug = "Error: " + E.ToString();

                            i++;
                        }
                    }
                    else
                    {
                        Vec cg = CGMethod(J, E, Vmove, 1e-6 * E.Var.Count, iterationMax);

                        coords += cg;
                        this.UpdateMesh(coords);


                        J = ComputeJacobian(Constraints, Sim);
                        E = ComputeError(Constraints, Sim);
                        int i = 0;
                        while (i < rigidity && E.NormSquared() > 1e-6 * E.Var.Count)
                        {
                            Vmove = 0 * Vmove;
                            cg = CGMethod(J, E, Vmove, E.NormSquared() / 100, iterationMax);

                            // Start Compute Line Search/////////////////////////////////////////////
                            double GoldenRatio = ((Math.Sqrt(5.0) - 1.0) / 2.0);

                            // Compute Upper and Lower Bound
                            Vec lb = 0 * cg;
                            Vec ub = 1 * cg;

                            Vec x1, x2; // 内分点（「１：φ」と「φ：１」に分割する点）
                            double e1, e2; // 内分点での誤差値

                            // 内分点を計算
                            x1 = (1.0f / (GoldenRatio + 1.0)) * (GoldenRatio * lb + ub);
                            UpdateMesh(coords + x1);
                            E = ComputeError(Constraints, Sim);
                            e1 = E.NormSquared();

                            x2 = (1.0 / (GoldenRatio + 1.0)) * (lb + GoldenRatio * ub);
                            UpdateMesh(coords + x2);
                            E = ComputeError(Constraints, Sim);
                            e2 = E.NormSquared();


                            for (int j = 0; j < 5; j++)
                            {
                                if (e1 < e2)
                                {
                                    ub = x2;
                                    x2 = x1;
                                    e2 = e1;
                                    x1 = (1.0 / (GoldenRatio + 1.0)) * (ub - lb) + lb;
                                    this.UpdateMesh(coords + x1);
                                    E = ComputeError(Constraints, Sim);
                                    e1 = E.NormSquared();
                                }
                                else
                                {
                                    lb = x1;
                                    x1 = x2;
                                    e1 = e2;
                                    x2 = (1.0 / GoldenRatio) * (ub - lb) + lb;
                                    this.UpdateMesh(coords + x2);
                                    E = ComputeError(Constraints, Sim);
                                    e2 = E.NormSquared();
                                }
                            }
                            Vec ls = (1.0 / 2.0) * (lb + ub);
                            coords += ls;
                            // End Compute Line Serch//////////////////////////////////////////////////////////////////

                            this.UpdateMesh(coords);
                            J = ComputeJacobian(Constraints, Sim);
                            E = ComputeError(Constraints, Sim);
                            i++;
                        }
                    }
                    debug = "Error: " + E.NormSquared().ToString();
                }
                else
                {
                    //simlationモードでもなく幾何拘束もないときはgrabのベクトルそのまま使って更新
                    coords += -1 * Vmove;
                    this.UpdateMesh(coords);
                }

            }
            else
            {
                this.isOn = false;
                this.timer.Stop();
            }

            //出力セット
            DA.SetData(0, this.cmesh);
            DA.SetData(1, debug);

        }

        private CRS ComputeJacobian(List<Constraint> c, bool simulation)
        {
            CRS Jaco = new CRS();

            if (simulation)
            {
                Jaco.Merge(ComputeEdgeLengthJacobian());
                if (QuadFlatOn)
                {
                    Jaco.Merge(ComputeQuadFlatJacobian());
                }
                if (FoldBlockOn)
                {
                    Jaco.Merge(ComputeMountainIntersectPenaltyJacobian());
                    Jaco.Merge(ComputeValleyIntersectPenaltyJacobian());
                }
                for (int i = 0; i < c.Count; i++)
                {
                    if (c[i].IsForRigidMode())
                    {
                        Jaco.Merge(c[i].Jacobian(this.cmesh));
                    }
                }
            }


            else if (c.Count != 0)
            {
                if (QuadFlatOn)
                {
                    Jaco.Merge(ComputeQuadFlatJacobian());
                }
                if (FoldBlockOn)
                {
                    Jaco.Merge(ComputeMountainIntersectPenaltyJacobian());
                    Jaco.Merge(ComputeValleyIntersectPenaltyJacobian());
                }
                for (int i = 0; i < c.Count; i++)
                {
                    Jaco.Merge(c[i].Jacobian(this.cmesh));
                }
            }

            return Jaco;
        }

        private CRS ComputeFoldJacobian(List<Constraint> c, bool simulation)
        {
            CRS Jaco = new CRS();

            if (simulation)
            {
                Jaco.Merge(ComputeEdgeLengthJacobian());
                Jaco.Merge(ComputeQuadFlatJacobian());

                for (int i = 0; i < c.Count; i++)
                {
                    if (c[i].IsForRigidMode())
                    {
                        Jaco.Merge(c[i].Jacobian(this.cmesh));
                    }
                }
            }

            else if (c.Count != 0)
            {
                for (int i = 0; i < c.Count; i++)
                {
                    Jaco.Merge(c[i].Jacobian(this.cmesh));
                }
            }

            return Jaco;
        }

        private CRS ComputeEdgeLengthJacobian()
        {
            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();


            for (int i = 0; i < this.mesh.TopologyEdges.Count; i++)
            {
                Rhino.IndexPair ind = this.mesh.TopologyEdges.GetTopologyVertices(i);
                Point3d a = this.mesh.Vertices[ind.I];
                Point3d b = this.mesh.Vertices[ind.J];

                for (int j = 0; j < 3; j++)
                {
                    var.Add(((double)a[j] - (double)b[j]) / this.EdgeLengthSquared[i]);
                    cind.Add(3 * ind.I + j);
                    rind.Add(i);

                    var.Add(((double)b[j] - (double)a[j]) / this.EdgeLengthSquared[i]);
                    cind.Add(3 * ind.J + j);
                    rind.Add(i);
                }
            }

            CRS Jaco = new CRS(var, rind, cind, this.mesh.TopologyEdges.Count, this.mesh.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeQuadFlatJacobian()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();

            MeshVertexList vert = m.Vertices;

            if (cm.triangulated_edges == null)
            {
                return null;
            }

            for (int e_ind = 0; e_ind < cm.triangulated_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = cm.triangulated_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.triangulated_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.triangulated_face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.triangulated_face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_triangulated_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vq = vert[q] - vert[v];
                /// Compute Jacobian
                for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                {
                    if (v_ind == p)
                    {
                        Vector3d delS_delX_p = Vector3d.CrossProduct(vec_uq, vec_uv) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_v = new List<double>();
                        vars_v.Add(delS_delX_p.X);
                        vars_v.Add(delS_delX_p.Y);
                        vars_v.Add(delS_delX_p.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_v[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == q)
                    {
                        Vector3d delS_delX_q = Vector3d.CrossProduct(vec_uv, vec_up) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_q = new List<double>();
                        vars_q.Add(delS_delX_q.X);
                        vars_q.Add(delS_delX_q.Y);
                        vars_q.Add(delS_delX_q.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_q[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == u)
                    {
                        Vector3d delS_delX_u = Vector3d.CrossProduct(vec_vq, vec_vp) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_u = new List<double>();
                        vars_u.Add(delS_delX_u.X);
                        vars_u.Add(delS_delX_u.Y);
                        vars_u.Add(delS_delX_u.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_u[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == v)
                    {
                        Vector3d delS_delX_v = Vector3d.CrossProduct(vec_up, vec_uq) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_v = new List<double>();
                        vars_v.Add(delS_delX_v.X);
                        vars_v.Add(delS_delX_v.Y);
                        vars_v.Add(delS_delX_v.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_v[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                }
            }
            CRS Jaco = new CRS(var, rind, cind, cm.triangulated_edges.Count, m.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeMountainFoldAngleJacobian()
        {
            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();
            this.mesh.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = this.mesh.Vertices;

            for (int e_ind = 0; e_ind < this.cmesh.mountain_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = this.cmesh.mountain_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = this.cmesh.mountain_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = this.mesh.Faces[P];
                MeshFace face_Q = this.mesh.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute normals
                Vector3d normal_P = this.mesh.FaceNormals[P];
                Vector3d normal_Q = this.mesh.FaceNormals[Q];

                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vu = vert[u] - vert[v];
                Vector3d vec_uq = vert[q] - vert[u];
                Vector3d vec_vq = vert[q] - vert[v];

                double volume = vec_uq * Vector3d.CrossProduct(vec_uv, vec_up);

                if (volume > 0)
                {
                    // Compute h_P & cot_Pu
                    double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                    double cos_Pu = (vec_up * vec_uv) / (vec_up.Length * vec_uv.Length);
                    double cot_Pu = cos_Pu / sin_Pu;
                    double len_up = vec_up.Length;
                    double h_P = len_up * sin_Pu;
                    /// Compute cot_Pv
                    double sin_Pv = (Vector3d.CrossProduct(vec_vp, vec_vu) / (vec_vp.Length * vec_vu.Length)).Length;
                    double cos_Pv = (vec_vp * vec_vu) / (vec_vp.Length * vec_vu.Length);
                    double cot_Pv = cos_Pv / sin_Pv;
                    /// Compute h_Q & cot_Qu
                    double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                    double cos_Qu = (vec_uq * vec_uv) / (vec_uq.Length * vec_uv.Length);
                    double cot_Qu = cos_Qu / sin_Qu;
                    double len_uq = vec_uq.Length;
                    double h_Q = len_uq * sin_Qu;
                    /// Compute cot_Qv
                    double sin_Qv = (Vector3d.CrossProduct(vec_vq, vec_vu) / (vec_vq.Length * vec_vu.Length)).Length;
                    double cos_Qv = vec_vq * vec_vu / (vec_vq.Length * vec_vu.Length);
                    double cot_Qv = cos_Qv / sin_Qv;
                    List<double> normal_P_list = new List<double>();
                    List<double> normal_Q_list = new List<double>();
                    normal_P_list.Add(normal_P.X);
                    normal_P_list.Add(normal_P.Y);
                    normal_P_list.Add(normal_P.Z);
                    normal_Q_list.Add(normal_Q.X);
                    normal_Q_list.Add(normal_Q.Y);
                    normal_Q_list.Add(normal_Q.Z);
                    /// Compute coefficients
                    double co_pv = (-1 * cot_Pv) / (cot_Pu + cot_Pv);
                    double co_qv = (-1 * cot_Qv) / (cot_Qu + cot_Qv);
                    double co_pu = (-1 * cot_Pu) / (cot_Pu + cot_Pv);
                    double co_qu = (-1 * cot_Qu) / (cot_Qu + cot_Qv);
                    /// Compute Jacobian
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(normal_P_list[x] / (h_P)));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(normal_Q_list[x] / (h_Q)));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)((co_pv * (normal_P_list[x] / h_P) + co_qv * (normal_Q_list[x] / h_Q))));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)((co_pu * (normal_P_list[x] / h_P) + co_qu * (normal_Q_list[x] / h_Q))));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }
                else
                {
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }

            }
            CRS Jaco = new CRS(var, rind, cind, this.cmesh.mountain_edges.Count, this.mesh.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeValleyFoldAngleJacobian()
        {
            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();
            this.mesh.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = this.mesh.Vertices;

            for (int e_ind = 0; e_ind < this.cmesh.valley_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = this.cmesh.valley_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = this.cmesh.valley_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = this.mesh.Faces[P];
                MeshFace face_Q = this.mesh.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute normals
                Vector3d normal_P = this.mesh.FaceNormals[P];
                Vector3d normal_Q = this.mesh.FaceNormals[Q];

                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vu = vert[u] - vert[v];
                Vector3d vec_uq = vert[q] - vert[u];
                Vector3d vec_vq = vert[q] - vert[v];

                double volume = vec_uq * Vector3d.CrossProduct(vec_uv, vec_up);

                if (volume < 0)
                {
                    // Compute h_P & cot_Pu
                    double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                    double cos_Pu = (vec_up * vec_uv) / (vec_up.Length * vec_uv.Length);
                    double cot_Pu = cos_Pu / sin_Pu;
                    double len_up = vec_up.Length;
                    double h_P = len_up * sin_Pu;
                    /// Compute cot_Pv
                    double sin_Pv = (Vector3d.CrossProduct(vec_vp, vec_vu) / (vec_vp.Length * vec_vu.Length)).Length;
                    double cos_Pv = (vec_vp * vec_vu) / (vec_vp.Length * vec_vu.Length);
                    double cot_Pv = cos_Pv / sin_Pv;
                    /// Compute h_Q & cot_Qu
                    double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                    double cos_Qu = (vec_uq * vec_uv) / (vec_uq.Length * vec_uv.Length);
                    double cot_Qu = cos_Qu / sin_Qu;
                    double len_uq = vec_uq.Length;
                    double h_Q = len_uq * sin_Qu;
                    /// Compute cot_Qv
                    double sin_Qv = (Vector3d.CrossProduct(vec_vq, vec_vu) / (vec_vq.Length * vec_vu.Length)).Length;
                    double cos_Qv = vec_vq * vec_vu / (vec_vq.Length * vec_vu.Length);
                    double cot_Qv = cos_Qv / sin_Qv;
                    List<double> normal_P_list = new List<double>();
                    List<double> normal_Q_list = new List<double>();
                    normal_P_list.Add(normal_P.X);
                    normal_P_list.Add(normal_P.Y);
                    normal_P_list.Add(normal_P.Z);
                    normal_Q_list.Add(normal_Q.X);
                    normal_Q_list.Add(normal_Q.Y);
                    normal_Q_list.Add(normal_Q.Z);
                    /// Compute coefficients
                    double co_pv = (-1 * cot_Pv) / (cot_Pu + cot_Pv);
                    double co_qv = (-1 * cot_Qv) / (cot_Qu + cot_Qv);
                    double co_pu = (-1 * cot_Pu) / (cot_Pu + cot_Pv);
                    double co_qu = (-1 * cot_Qu) / (cot_Qu + cot_Qv);
                    /// Compute Jacobian
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(normal_P_list[x] / (h_P)));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(normal_Q_list[x] / (h_Q)));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)((co_pv * (normal_P_list[x] / h_P) + co_qv * (normal_Q_list[x] / h_Q))));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)((co_pu * (normal_P_list[x] / h_P) + co_qu * (normal_Q_list[x] / h_Q))));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }
                else
                {
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }

            }
            CRS Jaco = new CRS(var, rind, cind, this.cmesh.valley_edges.Count, this.mesh.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeMountainIntersectPenaltyJacobian()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();

            MeshVertexList vert = m.Vertices;

            if (cm.mountain_edges == null)
            {
                return null;
            }

            for (int e_ind = 0; e_ind < cm.mountain_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = cm.mountain_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.mountain_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.mountain_face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.mountain_face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_mountain_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }

                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vq = vert[q] - vert[v];

                double volume = vec_up * Vector3d.CrossProduct(vec_uq, vec_uv);

                // 折角が [0, π] の範囲内のときヤコビアン計算
                if (volume > 0)
                {
                    /// Compute Jacobian
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            Vector3d delS_delX_p = Vector3d.CrossProduct(vec_uq, vec_uv) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_v = new List<double>();
                            vars_v.Add(delS_delX_p.X);
                            vars_v.Add(delS_delX_p.Y);
                            vars_v.Add(delS_delX_p.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_v[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            Vector3d delS_delX_q = Vector3d.CrossProduct(vec_uv, vec_up) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_q = new List<double>();
                            vars_q.Add(delS_delX_q.X);
                            vars_q.Add(delS_delX_q.Y);
                            vars_q.Add(delS_delX_q.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_q[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            Vector3d delS_delX_u = Vector3d.CrossProduct(vec_vq, vec_vp) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_u = new List<double>();
                            vars_u.Add(delS_delX_u.X);
                            vars_u.Add(delS_delX_u.Y);
                            vars_u.Add(delS_delX_u.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_u[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            Vector3d delS_delX_v = Vector3d.CrossProduct(vec_up, vec_uq) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_v = new List<double>();
                            vars_v.Add(delS_delX_v.X);
                            vars_v.Add(delS_delX_v.Y);
                            vars_v.Add(delS_delX_v.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_v[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }
                /*
                else
                {
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }
                */

            }
            CRS Jaco = new CRS(var, rind, cind, cm.mountain_edges.Count, m.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeValleyIntersectPenaltyJacobian()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();

            MeshVertexList vert = m.Vertices;

            if (cm.mountain_edges == null)
            {
                return null;
            }

            for (int e_ind = 0; e_ind < cm.valley_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = cm.valley_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.valley_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.valley_face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.valley_face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_valley_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }

                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vq = vert[q] - vert[v];

                double volume = vec_up * Vector3d.CrossProduct(vec_uq, vec_uv);

                // 折角が [-π, 0] の範囲内のときヤコビアン計算
                if (volume < 0)
                {
                    /// Compute Jacobian
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            Vector3d delS_delX_p = Vector3d.CrossProduct(vec_uq, vec_uv) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_v = new List<double>();
                            vars_v.Add(delS_delX_p.X);
                            vars_v.Add(delS_delX_p.Y);
                            vars_v.Add(delS_delX_p.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_v[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            Vector3d delS_delX_q = Vector3d.CrossProduct(vec_uv, vec_up) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_q = new List<double>();
                            vars_q.Add(delS_delX_q.X);
                            vars_q.Add(delS_delX_q.Y);
                            vars_q.Add(delS_delX_q.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_q[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            Vector3d delS_delX_u = Vector3d.CrossProduct(vec_vq, vec_vp) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_u = new List<double>();
                            vars_u.Add(delS_delX_u.X);
                            vars_u.Add(delS_delX_u.Y);
                            vars_u.Add(delS_delX_u.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_u[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            Vector3d delS_delX_v = Vector3d.CrossProduct(vec_up, vec_uq) / (h_P_initial * h_Q_initial * len_e);
                            List<double> vars_v = new List<double>();
                            vars_v.Add(delS_delX_v.X);
                            vars_v.Add(delS_delX_v.Y);
                            vars_v.Add(delS_delX_v.Z);
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add((double)(vars_v[x]));
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }
                /*
                else
                {
                    for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                    {
                        if (v_ind == p)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0f);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == q)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0f);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == u)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0f);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                        if (v_ind == v)
                        {
                            for (int x = 0; x < 3; x++)
                            {
                                var.Add(0.0f);
                                cind.Add(3 * v_ind + x);
                                rind.Add(e_ind);
                            }
                        }
                    }
                }
                */
            }
            CRS Jaco = new CRS(var, rind, cind, cm.valley_edges.Count, m.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeFoldAngleJacobian()
        {
            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();
            this.mesh.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = this.mesh.Vertices;

            for (int e_ind = 0; e_ind < this.cmesh.inner_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = this.cmesh.inner_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = this.cmesh.face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = this.mesh.Faces[P];
                MeshFace face_Q = this.mesh.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute normals
                Vector3d normal_P = this.mesh.FaceNormals[P];
                Vector3d normal_Q = this.mesh.FaceNormals[Q];
                /// Compute h_P & cot_Pu
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                double cos_Pu = (vec_up * vec_uv) / (vec_up.Length * vec_uv.Length);
                double cot_Pu = cos_Pu / sin_Pu;
                double len_up = vec_up.Length;
                double h_P = len_up * sin_Pu;
                /// Compute cot_Pv
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vu = vert[u] - vert[v];
                double sin_Pv = (Vector3d.CrossProduct(vec_vp, vec_vu) / (vec_vp.Length * vec_vu.Length)).Length;
                double cos_Pv = (vec_vp * vec_vu) / (vec_vp.Length * vec_vu.Length);
                double cot_Pv = cos_Pv / sin_Pv;
                /// Compute h_Q & cot_Qu
                Vector3d vec_uq = vert[q] - vert[u];
                double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                double cos_Qu = (vec_uq * vec_uv) / (vec_uq.Length * vec_uv.Length);
                double cot_Qu = cos_Qu / sin_Qu;
                double len_uq = vec_uq.Length;
                double h_Q = len_uq * sin_Qu;
                /// Compute cot_Qv
                Vector3d vec_vq = vert[q] - vert[v];
                double sin_Qv = (Vector3d.CrossProduct(vec_vq, vec_vu) / (vec_vq.Length * vec_vu.Length)).Length;
                double cos_Qv = vec_vq * vec_vu / (vec_vq.Length * vec_vu.Length);
                double cot_Qv = cos_Qv / sin_Qv;
                List<double> normal_P_list = new List<double>();
                List<double> normal_Q_list = new List<double>();
                normal_P_list.Add(normal_P.X);
                normal_P_list.Add(normal_P.Y);
                normal_P_list.Add(normal_P.Z);
                normal_Q_list.Add(normal_Q.X);
                normal_Q_list.Add(normal_Q.Y);
                normal_Q_list.Add(normal_Q.Z);
                /// Compute coefficients
                double co_pv = (-1 * cot_Pv) / (cot_Pu + cot_Pv);
                double co_qv = (-1 * cot_Qv) / (cot_Qu + cot_Qv);
                double co_pu = (-1 * cot_Pu) / (cot_Pu + cot_Pv);
                double co_qu = (-1 * cot_Qu) / (cot_Qu + cot_Qv);
                /// Compute Jacobian
                for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                {
                    if (v_ind == p)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(normal_P_list[x] / (h_P)));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == q)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(normal_Q_list[x] / (h_Q)));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == u)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)((co_pv * (normal_P_list[x] / h_P) + co_qv * (normal_Q_list[x] / h_Q))));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == v)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)((co_pu * (normal_P_list[x] / h_P) + co_qu * (normal_Q_list[x] / h_Q))));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                }
            }
            CRS Jaco = new CRS(var, rind, cind, this.cmesh.inner_edges.Count, this.mesh.Vertices.Count * 3);

            return Jaco;
        }

        private CRS ComputeNormalizedFoldAngleJacobian()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> var = new List<double>();
            List<int> cind = new List<int>();
            List<int> rind = new List<int>();

            MeshVertexList vert = m.Vertices;

            if (cm.inner_edges == null)
            {
                return null;
            }

            for (int e_ind = 0; e_ind < cm.inner_edges.Count; e_ind++)
            {
                /// Register indices
                IndexPair edge_ind = cm.inner_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];
                Vector3d vec_vp = vert[p] - vert[v];
                Vector3d vec_vq = vert[q] - vert[v];
                /// Compute Jacobian
                for (int v_ind = 0; v_ind < vert.Count; v_ind++)
                {
                    if (v_ind == p)
                    {
                        Vector3d delS_delX_p = Vector3d.CrossProduct(vec_uq, vec_uv) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_v = new List<double>();
                        vars_v.Add(delS_delX_p.X);
                        vars_v.Add(delS_delX_p.Y);
                        vars_v.Add(delS_delX_p.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_v[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == q)
                    {
                        Vector3d delS_delX_q = Vector3d.CrossProduct(vec_uv, vec_up) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_q = new List<double>();
                        vars_q.Add(delS_delX_q.X);
                        vars_q.Add(delS_delX_q.Y);
                        vars_q.Add(delS_delX_q.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_q[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == u)
                    {
                        Vector3d delS_delX_u = Vector3d.CrossProduct(vec_vq, vec_vp) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_u = new List<double>();
                        vars_u.Add(delS_delX_u.X);
                        vars_u.Add(delS_delX_u.Y);
                        vars_u.Add(delS_delX_u.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_u[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                    if (v_ind == v)
                    {
                        Vector3d delS_delX_v = Vector3d.CrossProduct(vec_up, vec_uq) / (h_P_initial * h_Q_initial * len_e);
                        List<double> vars_v = new List<double>();
                        vars_v.Add(delS_delX_v.X);
                        vars_v.Add(delS_delX_v.Y);
                        vars_v.Add(delS_delX_v.Z);
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((double)(vars_v[x]));
                            cind.Add(3 * v_ind + x);
                            rind.Add(e_ind);
                        }
                    }
                }
            }
            CRS Jaco = new CRS(var, rind, cind, cm.inner_edges.Count, m.Vertices.Count * 3);

            return Jaco;
        }

        protected Vec ComputeError(List<Constraint> c, bool simulation)
        {
            Vec Err = new Vec();

            if (simulation)
            {
                Err.Merge(ComputeEdgeLengthError());
                if (QuadFlatOn)
                {
                    Err.Merge(ComputeQuadFlatError());
                }
                if (FoldBlockOn)
                {
                    Err.Merge(ComputeMountainIntersectPenalty());
                    Err.Merge(ComputeValleyIntersectPenalty());
                }
                for (int i = 0; i < c.Count; i++)
                {
                    if (c[i].IsForRigidMode())
                    {
                        Err.Merge(c[i].Error(this.cmesh));
                    }
                }
            }

            else if (c.Count != 0)
            {
                if (QuadFlatOn)
                {
                    Err.Merge(ComputeQuadFlatError());
                }
                if (FoldBlockOn)
                {
                    Err.Merge(ComputeMountainIntersectPenalty());
                    Err.Merge(ComputeValleyIntersectPenalty());
                }
                for (int i = 0; i < c.Count; i++)
                {
                    Err.Merge(c[i].Error(this.cmesh));
                }
            }

            return Err;
        }

        private Vec ComputeEdgeLengthError()
        {
            List<double> ans = new List<double>();
            for (int i = 0; i < this.mesh.TopologyEdges.Count; i++)
            {
                IndexPair ind = this.mesh.TopologyEdges.GetTopologyVertices(i);
                Point3d a = this.mesh.Vertices[ind.I];
                Point3d b = this.mesh.Vertices[ind.J];
                ans.Add(((double)a.DistanceToSquared(b) / this.EdgeLengthSquared[i] - 1) / 2);
            }
            Vec V_ans = new Vec(ans);

            return V_ans;
        }

        private Vec ComputeQuadFlatError()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> ans = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.triangulated_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.triangulated_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.triangulated_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.triangulated_face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.triangulated_face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_triangulated_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                // Compute Volume
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];

                double volume = vec_up * Vector3d.CrossProduct(vec_uq, vec_uv);
                double S_e = (double)(volume / (h_P_initial * h_Q_initial * len_e));
                ans.Add(S_e);
            }
            Vec V_ans = new Vec(ans);

            return V_ans;

        }

        private Vec ComputeMountainIntersectPenalty()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> ans = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.mountain_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.mountain_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.mountain_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.mountain_face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.mountain_face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_mountain_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                //Compute Volume
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];

                double volume = vec_up * Vector3d.CrossProduct(vec_uq, vec_uv);
                double S_e = (double)(volume / (h_P_initial * h_Q_initial * len_e));
                if (volume > 0)
                {
                    ans.Add(S_e);
                }
                /*
                else
                {
                    ans.Add(0.0);
                }
                */
            }
            Vec V_ans = new Vec(ans);

            return V_ans;

        }

        private Vec ComputeValleyIntersectPenalty()
        {
            CMesh cm = this.cmesh;
            Mesh m = cm.mesh;

            List<double> ans = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.valley_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.valley_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.valley_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;

                double h_P_initial = cm.valley_face_height_pairs[e_ind].Item1;
                double h_Q_initial = cm.valley_face_height_pairs[e_ind].Item2;
                double len_e = cm.length_of_valley_diagonal_edges[e_ind];

                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }

                //Compute Volume
                Vector3d vec_uv = vert[v] - vert[u];
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uq = vert[q] - vert[u];

                double volume = vec_up * Vector3d.CrossProduct(vec_uq, vec_uv);
                double S_e = (double)(volume / (h_P_initial * h_Q_initial * len_e));
                if (volume < 0)
                {
                    ans.Add(S_e);
                }
                /*
                else
                {
                    ans.Add(0.0);
                }
                */
            }
            Vec V_ans = new Vec(ans);

            return V_ans;

        }

        protected Vec CGMethod(CRS Jacobian, Vec b, Vec x, double threshold, int IterationMax)
        {

            int it = 0;
            Vec r = b - Jacobian * x;
            Vec p = Jacobian.TransposeAndMultiply(r);

            double alpha = 0;
            double beta = 0;

            while ((it < IterationMax) & r.NormSquared() > threshold)
            {
                Vec ATr = Jacobian.TransposeAndMultiply(r);
                alpha = ATr.NormSquared() / (Jacobian * p).NormSquared();
                x = x + alpha * p;
                r = b - Jacobian * x;
                beta = Jacobian.TransposeAndMultiply(r).NormSquared() / ATr.NormSquared();
                p = Jacobian.TransposeAndMultiply(r) + beta * p;
                it++;
            }

            return -1 * x;
        }

        protected Vec CGMethodForSymmetricPositiveDefinite(SparseMatrix A, DenseVector b, double threshold, int IterationMax)
        {

            /// Require:
            ///     threthold >= 0
            ///     A; symmetry-definite n*n matrix
            /// Ensure:
            ///     x_i = (A^(-1))b     for i = n-1

            DenseVector x = new DenseVector(A.ColumnCount);

            int it = 0;
            DenseVector r = b;
            DenseVector p = r;

            double alpha = 0;
            double beta = 0;

            while ((it < IterationMax) & (r.L2Norm() > threshold))
            {
                double rTr = (double)Math.Pow(r.L2Norm(), 2);
                double pTAp = p * (A * p);

                alpha = rTr / pTAp;
                x = x + alpha * p;
                r = b - (DenseVector)(A * x);
                beta = (double)Math.Pow(r.L2Norm(), 2) / rTr;
                p = r + beta * p;
                it++;
            }

            Vec X = new Vec();
            X = Vec.TransformFromDennseVector(x);

            return -1 * X;
        }

        private void UpdateMesh(Vec v)
        {
            mesh.Vertices.Destroy();
            for (int i = 0; i < (v.Var.Count / 3); i++)
            {
                mesh.Vertices.Add(v.Var[3 * i], v.Var[3 * i + 1], v.Var[3 * i + 2]);
            }
            mesh.Compact();
            mesh.Normals.Destroy();
            mesh.Normals.ComputeNormals();

        }

        private void UpdateEdgeLengthSquared()
        {
            List<double> temp = new List<double>();

            for (int i = 0; i < this.mesh.TopologyEdges.Count; i++)
            {
                Rhino.IndexPair ind = this.mesh.TopologyEdges.GetTopologyVertices(i);
                Point3d a = this.mesh.Vertices[ind.I];
                Point3d b = this.mesh.Vertices[ind.J];
                temp.Add(a.DistanceToSquared(b));
            }

            this.EdgeLengthSquared = temp;
        }

        private void UpdateFacePairBasicInfo(CMesh cm)
        {
            Mesh m = cm.mesh;

            List<Tuple<double, double>> face_heignt_pairs = new List<Tuple<double, double>>();
            List<double> edge_length_between_face_pairs = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.inner_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.inner_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute h_P & cot_Pu
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                double len_up = (vec_up - vec_uv).Length;
                double h_P = len_up * sin_Pu;
                /// Compute h_Q & cot_Qu
                Vector3d vec_uq = vert[q] - vert[u];
                double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                double len_uq = (vec_uq - vec_uv).Length;
                double h_Q = len_uq * sin_Qu;
                // Compute len_uv
                double len_uv = vec_uv.Length;
                // Set Tuple<h_P, h_Q>
                Tuple<double, double> face_height_pair = new Tuple<double, double>(h_P, h_Q);
                face_heignt_pairs.Add(face_height_pair);
                edge_length_between_face_pairs.Add(len_uv);
            }
            cm.face_height_pairs = face_heignt_pairs;
            cm.length_of_diagonal_edges = edge_length_between_face_pairs;
        }

        private void UpdateTriangulatedFacePairBasicInfo(CMesh cm)
        {
            Mesh m = cm.mesh;

            List<Tuple<double, double>> face_heignt_pairs = new List<Tuple<double, double>>();
            List<double> edge_length_between_face_pairs = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.triangulated_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.triangulated_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.triangulated_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute h_P & cot_Pu
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                double len_up = (vec_up - vec_uv).Length;
                double h_P = len_up * sin_Pu;
                /// Compute h_Q & cot_Qu
                Vector3d vec_uq = vert[q] - vert[u];
                double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                double len_uq = (vec_uq - vec_uv).Length;
                double h_Q = len_uq * sin_Qu;
                // Compute len_uv
                double len_uv = vec_uv.Length;
                // Set Tuple<h_P, h_Q>
                Tuple<double, double> face_height_pair = new Tuple<double, double>(h_P, h_Q);
                face_heignt_pairs.Add(face_height_pair);
                edge_length_between_face_pairs.Add(len_uv);
            }
            cm.triangulated_face_height_pairs = face_heignt_pairs;
            cm.length_of_triangulated_diagonal_edges = edge_length_between_face_pairs;
        }

        private void UpdateMountainFacePairBasicInfo(CMesh cm)
        {
            Mesh m = cm.mesh;

            List<Tuple<double, double>> face_heignt_pairs = new List<Tuple<double, double>>();
            List<double> edge_length_between_face_pairs = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.mountain_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.mountain_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.mountain_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute h_P & cot_Pu
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                double len_up = (vec_up - vec_uv).Length;
                double h_P = len_up * sin_Pu;
                /// Compute h_Q & cot_Qu
                Vector3d vec_uq = vert[q] - vert[u];
                double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                double len_uq = (vec_uq - vec_uv).Length;
                double h_Q = len_uq * sin_Qu;
                // Compute len_uv
                double len_uv = vec_uv.Length;
                // Set Tuple<h_P, h_Q>
                Tuple<double, double> face_height_pair = new Tuple<double, double>(h_P, h_Q);
                face_heignt_pairs.Add(face_height_pair);
                edge_length_between_face_pairs.Add(len_uv);
            }
            cm.mountain_face_height_pairs = face_heignt_pairs;
            cm.length_of_mountain_diagonal_edges = edge_length_between_face_pairs;
        }

        private void UpdateValleyFacePairBasicInfo(CMesh cm)
        {
            Mesh m = cm.mesh;

            List<Tuple<double, double>> face_heignt_pairs = new List<Tuple<double, double>>();
            List<double> edge_length_between_face_pairs = new List<double>();

            m.FaceNormals.ComputeFaceNormals();

            MeshVertexList vert = m.Vertices;

            for (int e_ind = 0; e_ind < cm.valley_edges.Count; e_ind++)
            {
                // Register indices
                IndexPair edge_ind = cm.valley_edges[e_ind];
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.valley_face_pairs[e_ind];
                int P = face_ind.I;
                int Q = face_ind.J;

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;
                for (int i = 0; i < 3; i++)
                {
                    if (!edge_ind.Contains(face_P[i]))
                    {
                        p = face_P[i];
                    }
                    if (!edge_ind.Contains(face_Q[i]))
                    {
                        q = face_Q[i];
                    }
                }
                /// Compute h_P & cot_Pu
                Vector3d vec_up = vert[p] - vert[u];
                Vector3d vec_uv = vert[v] - vert[u];
                double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                double len_up = (vec_up - vec_uv).Length;
                double h_P = len_up * sin_Pu;
                /// Compute h_Q & cot_Qu
                Vector3d vec_uq = vert[q] - vert[u];
                double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                double len_uq = (vec_uq - vec_uv).Length;
                double h_Q = len_uq * sin_Qu;
                // Compute len_uv
                double len_uv = vec_uv.Length;
                // Set Tuple<h_P, h_Q>
                Tuple<double, double> face_height_pair = new Tuple<double, double>(h_P, h_Q);
                face_heignt_pairs.Add(face_height_pair);
                edge_length_between_face_pairs.Add(len_uv);
            }
            cm.valley_face_height_pairs = face_heignt_pairs;
            cm.length_of_valley_diagonal_edges = edge_length_between_face_pairs;
        }

        private SparseMatrix ComputeFoldMotionMatrix(CRS FoldAngleJacobian, CRS Jacobian, double weight)
        {
            SparseMatrix foldjacosparse = FoldAngleJacobian.TransformToSparseMatrix();
            SparseMatrix jacosparse = Jacobian.TransformToSparseMatrix();

            SparseMatrix foldjaco_squared = (SparseMatrix)(foldjacosparse.Transpose() * foldjacosparse);
            SparseMatrix jaco_squared = (SparseMatrix)(jacosparse.Transpose() * jacosparse);
            SparseMatrix sm = (SparseMatrix)((1.0 / weight) * foldjaco_squared + (((weight - 1.0) / weight) * jaco_squared));

            return sm;
        }

        private DenseVector ComputeFoldMotionVector(CRS FoldAngleJacobian, DenseVector InitialFoldAngleVector)
        {
            SparseMatrix foldjacosparse = FoldAngleJacobian.TransformToSparseMatrix();

            DenseVector dv = (DenseVector)(foldjacosparse.Transpose() * InitialFoldAngleVector);

            return dv;
        }

        private DenseVector ComputeInitialFoldAngleVector(double foldspeed)
        {
            DenseVector dv;
            List<double> foldang = new List<double>();
            foldang = this.cmesh.GetFoldAngle();
            double[] driving_force = new double[this.cmesh.inner_edge_assignment.Count];

            for (int i = 0; i < this.cmesh.inner_edge_assignment.Count; i++)
            {
                if (cmesh.inner_edge_assignment[i] == 'V')
                {
                    driving_force[i] = (double)(foldspeed * Math.Cos(foldang[i] / 2));
                }
                else if (cmesh.inner_edge_assignment[i] == 'M')
                {
                    driving_force[i] = (double)(-foldspeed * Math.Cos(foldang[i] / 2));
                }
                else
                {
                    driving_force[i] = 0;
                }
            }
            dv = DenseVector.OfArray(driving_force);

            return dv;
        }

        private DenseVector ComputeInitialFoldAngleVector2(double foldspeed)
        {
            DenseVector dv;
            List<double> foldang = new List<double>();
            foldang = this.cmesh.GetFoldAngle();
            double[] driving_force = new double[this.cmesh.inner_edge_assignment.Count];

            for (int i = 0; i < this.cmesh.inner_edge_assignment.Count; i++)
            {
                if (cmesh.inner_edge_assignment[i] == 'V')
                {
                    driving_force[i] = (double)(-foldspeed * Math.Pow(Math.Cos(Math.PI / 2 - Math.Abs(foldang[i]) / 2), 1));
                }
                else if (cmesh.inner_edge_assignment[i] == 'M')
                {
                    driving_force[i] = (double)(foldspeed * Math.Pow(Math.Cos(Math.PI / 2 - Math.Abs(foldang[i]) / 2), 1));
                }
                else
                {
                    driving_force[i] = 0;
                }
            }
            dv = DenseVector.OfArray(driving_force);

            return dv;
        }

        public class MyMouseCallback : Rhino.UI.MouseCallback
        {
            protected override void OnMouseDown(Rhino.UI.MouseCallbackEventArgs e)
            {
                e.Cancel = true;
                base.OnMouseDown(e);
            }
        }

        public class MyMouseCallbackOff : Rhino.UI.MouseCallback
        {
            protected override void OnMouseDown(Rhino.UI.MouseCallbackEventArgs e)
            {
                e.Cancel = false;
                base.OnMouseDown(e);
            }
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                return Crane.Properties.Resources.sol;
            }
        }

        private void tick(object myObject, EventArgs myeventArgs)
        {
            if (this.isOn && !(this.Locked))
            {
                ExpireSolution(true);
            }

        }

        public override Guid ComponentGuid
        {
            get { return new Guid("ddf37f86-882d-47de-b407-c348a7d04ad2"); }
        }

        /// <summary>
        /// コンポーネントのカスタムの中身のクラス
        /// </summary>

    }

    public class Attributes_Custom : GH_ComponentAttributes
    {
        public Attributes_Custom(GH_Component owner) : base(owner) { }
        protected override void Layout()
        {
            base.Layout();

            // コンポーネントのサイズを取得し、ボタン分の長さをプラス(+44)する
            Rectangle rec0 = GH_Convert.ToRectangle(Bounds);
            rec0.Height += 154;
            // 22の高さの長方形を配置する
            Rectangle rec1 = rec0;
            rec1.Y = rec1.Bottom - 154;
            rec1.Height = 22;
            rec1.Inflate(-2, -2);


            Rectangle rec2 = rec0;
            rec2.Y = rec0.Bottom - 132;
            rec2.Height = 22;
            rec2.Inflate(-2, -2);

            Rectangle rec3 = rec0;
            rec3.Y = rec0.Bottom - 110;
            rec3.Height = 22;
            rec3.Inflate(-2, -2);

            Rectangle rec4 = rec0;
            rec4.Y = rec0.Bottom - 88;
            rec4.Height = 22;
            rec4.Inflate(-2, -2);

            Rectangle rec5 = rec0;
            rec5.Y = rec0.Bottom - 66;
            rec5.Height = 22;
            rec5.Inflate(-2, -2);

            Rectangle rec6 = rec0;
            rec6.Y = rec0.Bottom - 44;
            rec6.Height = 22;
            rec6.Inflate(-2, -2);

            Rectangle rec7 = rec0;
            rec7.Y = rec0.Bottom - 22;
            rec7.Height = 22;
            rec7.Inflate(-2, -2);

            Bounds = rec0;
            ButtonBounds = rec1;
            ButtonBounds2 = rec2;
            ButtonBounds3 = rec3;
            ButtonBounds4 = rec4;
            ButtonBounds5 = rec5;
            ButtonBounds6 = rec6;
            ButtonBounds7 = rec7;
        }
        private Rectangle ButtonBounds { get; set; }
        private Rectangle ButtonBounds2 { get; set; }
        private Rectangle ButtonBounds3 { get; set; }
        private Rectangle ButtonBounds4 { get; set; }
        private Rectangle ButtonBounds5 { get; set; }
        private Rectangle ButtonBounds6 { get; set; }
        private Rectangle ButtonBounds7 { get; set; }

        protected override void Render(GH_Canvas canvas, Graphics graphics, GH_CanvasChannel channel)
        {
            base.Render(canvas, graphics, channel);
            CraneSolverComponent comp = this.Owner as CraneSolverComponent;
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds, ButtonBounds, GH_Palette.Black, "Push Button to Fold", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds2, ButtonBounds2, GH_Palette.Black, "Push Button to Unfold", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds3, ButtonBounds3, GH_Palette.Black, "Push Button to Reset", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds4, ButtonBounds4, GH_Palette.Black, "Solver: Off", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds5, ButtonBounds5, GH_Palette.Black, "Rigid Mode: Off", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds6, ButtonBounds6, GH_Palette.Black, "Quad Flat: Off", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (channel == GH_CanvasChannel.Objects)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds7, ButtonBounds7, GH_Palette.Black, "180 Fold Brocking: Off", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.Fold & !comp.Unfold)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds, ButtonBounds, GH_Palette.Black, "Folding", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.Fold & comp.Unfold)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds2, ButtonBounds2, GH_Palette.Black, "UnFolding", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.Reset)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds3, ButtonBounds3, GH_Palette.Black, "Resetting", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.SolverOn)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds4, ButtonBounds4, GH_Palette.Black, "Solver: On", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.RigidModeOn)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds5, ButtonBounds5, GH_Palette.Black, "Rigid Mode: On", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.QuadFlatOn)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds6, ButtonBounds6, GH_Palette.Black, "Quad Flat: On", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }
            if (comp.FoldBlockOn)
            {
                GH_Capsule button = GH_Capsule.CreateTextCapsule(ButtonBounds7, ButtonBounds7, GH_Palette.Black, "180 Fold Blocking: On", 2, 0);
                button.Render(graphics, Selected, Owner.Locked, false);
                button.Dispose();
            }

        }


        /// <summary>
        /// マウスをクリックしたときのイベントハンドラ
        /// </summary>
        public override GH_ObjectResponse RespondToMouseDown(GH_Canvas sender, GH_CanvasMouseEvent e)
        {
            // ButtonBoundsを押したときのイベント
            if (e.Button == MouseButtons.Left)
            {
                CraneSolverComponent comp = this.Owner as CraneSolverComponent;
                RectangleF rec = ButtonBounds;
                if (rec.Contains(e.CanvasLocation))
                {
                    comp.Fold = true;
                    return GH_ObjectResponse.Handled;
                }
                RectangleF rec2 = ButtonBounds2;
                if (rec2.Contains(e.CanvasLocation))
                {
                    comp.Fold = true;
                    comp.Unfold = true;
                    return GH_ObjectResponse.Handled;
                }
                RectangleF rec3 = ButtonBounds3;
                if (rec3.Contains(e.CanvasLocation))
                {
                    comp.Reset = true;
                    return GH_ObjectResponse.Handled;
                }
                RectangleF rec4 = ButtonBounds4;
                if (rec4.Contains(e.CanvasLocation))
                {
                    if (comp.SolverOn)
                    {
                        comp.SolverOn = false;
                        comp.ExpireSolution(true);
                    }
                    else if (!comp.SolverOn)
                    {
                        comp.SolverOn = true;
                        comp.ExpireSolution(true);
                    }
                    return GH_ObjectResponse.Handled;
                }
                RectangleF rec5 = ButtonBounds5;
                if (rec5.Contains(e.CanvasLocation))
                {
                    if (comp.RigidModeOn)
                    {
                        comp.RigidModeOn = false;
                        comp.ExpireSolution(true);
                    }
                    else if (!comp.RigidModeOn)
                    {
                        comp.RigidModeOn = true;
                        comp.ExpireSolution(true);
                    }
                    return GH_ObjectResponse.Handled;
                }
                RectangleF rec6 = ButtonBounds6;
                if (rec6.Contains(e.CanvasLocation))
                {
                    if (comp.QuadFlatOn)
                    {
                        comp.QuadFlatOn = false;
                        comp.ExpireSolution(true);
                    }
                    else if (!comp.QuadFlatOn)
                    {
                        comp.QuadFlatOn = true;
                        comp.ExpireSolution(true);
                    }
                    return GH_ObjectResponse.Handled;
                }
                RectangleF rec7 = ButtonBounds7;
                if (rec7.Contains(e.CanvasLocation))
                {
                    if (comp.FoldBlockOn)
                    {
                        comp.FoldBlockOn = false;
                        comp.ExpireSolution(true);
                    }
                    else if (!comp.FoldBlockOn)
                    {
                        comp.FoldBlockOn = true;
                        comp.ExpireSolution(true);
                    }
                    return GH_ObjectResponse.Handled;
                }
            }

            return base.RespondToMouseDown(sender, e);
        }
        public override GH_ObjectResponse RespondToMouseUp(GH_Canvas sender, GH_CanvasMouseEvent e)
        {
            CraneSolverComponent comp = this.Owner as CraneSolverComponent;
            RectangleF rec = ButtonBounds;
            if (rec.Contains(e.CanvasLocation))
            {
                comp.Fold = false;
                return GH_ObjectResponse.Handled;
            }
            RectangleF rec2 = ButtonBounds2;
            if (rec2.Contains(e.CanvasLocation))
            {
                comp.Fold = false;
                comp.Unfold = false;
                return GH_ObjectResponse.Handled;
            }
            RectangleF rec3 = ButtonBounds3;
            if (rec3.Contains(e.CanvasLocation) & comp.Reset == true)
            {
                comp.Reset = false;
                return GH_ObjectResponse.Handled;
            }
            RectangleF rec4 = ButtonBounds4;
            return base.RespondToMouseUp(sender, e);
        }
    }
}

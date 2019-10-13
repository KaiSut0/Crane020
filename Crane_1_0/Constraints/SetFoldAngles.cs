using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;

namespace Crane
{
    public class SetFoldAngles : Constraint
    {
        List<int> equal_fold_angle_edge_id = new List<int>();
        double fold_angle = 0;
        bool is_set_angle = false;
        /// <summary>
        /// Initializes a new instance of the EqualFoldAngle class.
        /// </summary>
        public SetFoldAngles()
          : base("SetFoldAngles", "SetFoldAngles",
              "Equal fold angle constraint between selected edges. You can also set fold angle of selected edges",
              "Crane", "Constraints")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddIntegerParameter("EdgeID", "EdgeID", "Input Indices of edge for equal length constraint", GH_ParamAccess.list);
            pManager.AddNumberParameter("SetAngle", "SetAngle", "", GH_ParamAccess.item);

            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Constraint", "C", "Equal length constraint between selected edges", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 値をセットするごとに public member をリフレッシュ
            equal_fold_angle_edge_id = new List<int>();
            fold_angle = 0;
            is_set_angle = false;

            // 値を取得
            if (!DA.GetDataList(0, equal_fold_angle_edge_id)) { return; }
            is_set_angle = DA.GetData(1, ref fold_angle);
            DA.SetData(0, this);
        }

        public override CRS Jacobian(CMesh cm)
        {
            // 等長拘束をかける辺の数取得
            int n = equal_fold_angle_edge_id.Count;

            List<double> var = new List<double>();
            List<int> rind = new List<int>();
            List<int> cind = new List<int>();


            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());
            Mesh m = cm.mesh;
            m.FaceNormals.ComputeFaceNormals();

            // メッシュの TopologyEdges 取得
            MeshTopologyEdgeList topo_edges = m.TopologyEdges;
            for (int i = 0; i < n; i++)
            {
                // Register indices
                int edge_index = equal_fold_angle_edge_id[i];
                IndexPair edge_ind = topo_edges.GetTopologyVertices(edge_index);
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.face_pairs[edge_index];
                int P = face_ind[0];
                int Q = face_ind[1];

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;
                for (int j = 0; j < 3; j++)
                {
                    if (!edge_ind.Contains(face_P[j]))
                    {
                        p = face_P[j];
                    }
                    if (!edge_ind.Contains(face_Q[j]))
                    {
                        q = face_Q[j];
                    }
                }
                /// Compute normals
                Vector3d normal_P = m.FaceNormals[P];
                Vector3d normal_Q = m.FaceNormals[Q];
                /// Compute h_P & cot_Pu
                Vector3d vec_up = verts[p] - verts[u];
                Vector3d vec_uv = verts[v] - verts[u];
                double sin_Pu = (Vector3d.CrossProduct(vec_up, vec_uv) / (vec_up.Length * vec_uv.Length)).Length;
                double cos_Pu = (vec_up * vec_uv) / (vec_up.Length * vec_uv.Length);
                double cot_Pu = cos_Pu / sin_Pu;
                double len_up = vec_up.Length;
                double h_P = len_up * sin_Pu;
                /// Compute cot_Pv
                Vector3d vec_vp = verts[p] - verts[v];
                Vector3d vec_vu = verts[u] - verts[v];
                double sin_Pv = (Vector3d.CrossProduct(vec_vp, vec_vu) / (vec_vp.Length * vec_vu.Length)).Length;
                double cos_Pv = (vec_vp * vec_vu) / (vec_vp.Length * vec_vu.Length);
                double cot_Pv = cos_Pv / sin_Pv;
                /// Compute h_Q & cot_Qu
                Vector3d vec_uq = verts[q] - verts[u];
                double sin_Qu = (Vector3d.CrossProduct(vec_uq, vec_uv) / (vec_uq.Length * vec_uv.Length)).Length;
                double cos_Qu = (vec_uq * vec_uv) / (vec_uq.Length * vec_uv.Length);
                double cot_Qu = cos_Qu / sin_Qu;
                double len_uq = vec_uq.Length;
                double h_Q = len_uq * sin_Qu;
                /// Compute cot_Qv
                Vector3d vec_vq = verts[q] - verts[v];
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
                for (int v_ind = 0; v_ind < verts.Count; v_ind++)
                {
                    if (v_ind == p)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((normal_P_list[x] / (h_P)));
                            cind.Add(3 * v_ind + x);
                            rind.Add(i);
                        }
                    }
                    if (v_ind == q)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add((normal_Q_list[x] / (h_Q)));
                            cind.Add(3 * v_ind + x);
                            rind.Add(i);
                        }
                    }
                    if (v_ind == u)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add(((co_pv * (normal_P_list[x] / h_P) + co_qv * (normal_Q_list[x] / h_Q))));
                            cind.Add(3 * v_ind + x);
                            rind.Add(i);
                        }
                    }
                    if (v_ind == v)
                    {
                        for (int x = 0; x < 3; x++)
                        {
                            var.Add(((co_pu * (normal_P_list[x] / h_P) + co_qu * (normal_Q_list[x] / h_Q))));
                            cind.Add(3 * v_ind + x);
                            rind.Add(i);
                        }
                    }
                }
            }

            CRS Jaco = new CRS(var, rind, cind, n, verts.Count * 3);

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            Mesh m = cm.mesh;
            List<Point3d> verts = new List<Point3d>(m.Vertices.ToPoint3dArray());
            List<double> foldang = new List<double>();

            // 等長拘束をかける辺の数取得
            int n = equal_fold_angle_edge_id.Count;

            // メッシュの TopologyEdges 取得
            MeshTopologyEdgeList topo_edges = m.TopologyEdges;

            // 法線方向計算
            m.FaceNormals.ComputeFaceNormals();

            for (int i = 0; i < n; i++)
            {
                double foldang_i = 0;
                /// Register indices
                int edge_index = equal_fold_angle_edge_id[i];
                IndexPair edge_ind = topo_edges.GetTopologyVertices(edge_index);
                int u = edge_ind.I;
                int v = edge_ind.J;
                IndexPair face_ind = cm.face_pairs[edge_index];
                int P = face_ind[0];
                int Q = face_ind[1];

                MeshFace face_P = m.Faces[P];
                MeshFace face_Q = m.Faces[Q];
                int p = 0;
                int q = 0;
                for (int j = 0; j < 3; j++)
                {
                    if (!edge_ind.Contains(face_P[j]))
                    {
                        p = face_P[j];
                    }
                    if (!edge_ind.Contains(face_Q[j]))
                    {
                        q = face_Q[j];
                    }
                }
                /// Compute normals
                Vector3d normal_P = m.FaceNormals[P];
                Vector3d normal_Q = m.FaceNormals[Q];
                /// cos(foldang_e) = n_i * n_j
                double cos_foldang_e = normal_P * normal_Q;
                /// sin(foldang_e) = n_i × n_j
                Vector3d vec_e = verts[u] - verts[v];
                vec_e.Unitize();
                double sin_foldang_e = Vector3d.CrossProduct(normal_P, normal_Q) * vec_e;
                if (sin_foldang_e >= 0)
                {
                    if (cos_foldang_e >= 1.0)
                    {
                        foldang_i = 0;
                    }
                    else if (cos_foldang_e <= -1.0)
                    {
                        foldang_i = Math.PI;
                    }
                    else
                    {
                        foldang_i = Math.Acos(cos_foldang_e);
                    }
                }
                else
                {
                    if (cos_foldang_e >= 1.0)
                    {
                        foldang_i = 0;
                    }
                    else if (cos_foldang_e <= -1.0)
                    {
                        foldang_i = -Math.PI;
                    }
                    else
                    {
                        foldang_i = -Math.Acos(cos_foldang_e);
                    }
                }
                if (is_set_angle)
                {
                    foldang_i -= fold_angle;
                }
                foldang.Add(foldang_i);
            }
            Vec ans = new Vec(foldang);

            return ans;
        }

        public override bool IsForRigidMode()
        {
            return true;
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return Crane.Properties.Resources.set_fold_angle;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("14178089-d39d-4fe1-9b71-2fccbda40ddd"); }
        }
    }
}
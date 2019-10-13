using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Drawing;

using Rhino;
using Rhino.Geometry;
using Rhino.Geometry.Collections;
using Grasshopper.Kernel;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;
using MathNet.Numerics.LinearAlgebra.Storage;

namespace Crane
{
    public class CRS
    {
        public List<double> Var;
        public List<int> C_Index;
        public List<int> R_Ptr;

        public int R_Size;
        public int C_Size;

        public CRS()
        {
            this.Var = new List<double>();
            this.C_Index = new List<int>();
            this.R_Ptr = new List<int>();
        }

        public CRS(List<double> _Var, List<int> _R_Index, List<int> _C_Index, int _R_Size, int _C_Size)
        {
            this.Var = new List<double>(_Var);
            this.C_Index = new List<int>(_C_Index);

            this.R_Size = _R_Size;
            this.C_Size = _C_Size;

            List<int> _R_Ptr = new List<int>();
            _R_Ptr.Add(0);
            for (int i = 1; i < _R_Index.Count; i++)
            {
                if (_R_Index[i] != _R_Index[i - 1])
                {
                    _R_Ptr.Add(i);
                }
            }
            _R_Ptr.Add(_R_Index.Count);

            this.R_Ptr = _R_Ptr;

        }

        public SparseMatrix TransformToSparseMatrix()
        {
            // Original CRS
            List<double> var0 = this.Var;
            List<int> cind0 = this.C_Index;
            List<int> rind0 = new List<int>();
            List<int> rptr0 = this.R_Ptr;

            // Get row index of original CRS
            for (int i = 0; i < rptr0.Count - 1; i++)
            {
                for (int j = 0; j < rptr0[i + 1] - rptr0[i]; j++)
                {
                    rind0.Add(i);
                }
            }

            // Get Sparse Matrix
            SparseMatrix sm;
            List<Tuple<int, int, double>> sm_tuple = new List<Tuple<int, int, double>>();
            for (int i = 0; i < this.Var.Count; i++)
            {
                Tuple<int, int, double> tuple = new Tuple<int, int, double>(rind0[i], cind0[i], var0[i]);
                sm_tuple.Add(tuple);
            }
            if (sm_tuple.Count != 0)
            {
                sm = SparseMatrix.OfIndexed(this.R_Size, this.C_Size, sm_tuple);
                return sm;
            }
            else
            {
                return null;
            }
        }

        public CRS Transpose()
        {
            // Original CRS
            List<double> var0 = this.Var;
            List<int> cind0 = this.C_Index;
            List<int> rind0 = new List<int>();
            List<int> rptr0 = this.R_Ptr;

            // Transposed CRS
            CRS Transposed = new CRS();
            double[] var = new double[var0.Count];
            int[] cind = new int[var0.Count];
            int[] rptr = new int[this.C_Size + 1];

            // Get row index of original CRS
            for (int i = 0; i < rptr0.Count - 1; i++)
            {
                for (int j = 0; j < rptr0[i + 1] - rptr0[i]; j++)
                {
                    rind0.Add(i);
                }
            }

            // Get column ptr of Transposed CRS
            for (int i = 0; i < var0.Count; i++)
            {
                rptr[cind0[i] + 1] += 1;
            }
            for (int i = 1; i < this.C_Size + 1; i++)
            {
                rptr[i] += rptr[i - 1];
            }

            // Get var and row of Transposed CRS
            List<int> nn = new List<int>(rptr);
            for (int i = 0; i < var0.Count; i++)
            {
                int x = nn[cind0[i]];
                nn[cind0[i]] += 1;
                var[x] = var0[i];
                cind[x] = rind0[i];
            }
            Transposed.Var.AddRange(var);
            Transposed.C_Index.AddRange(cind);
            Transposed.R_Ptr.AddRange(rptr);
            Transposed.C_Size = this.R_Size;
            Transposed.R_Size = this.C_Size;
            return Transposed;
        }

        public Vec TransposeAndMultiply(Vec V)
        {
            List<double> ans = new List<double>();
            for (int i = 0; i < this.C_Size; i++)
            {
                ans.Add(0);
            }

            for (int i = 0; i < this.R_Ptr.Count - 1; i++)
            {
                for (int j = this.R_Ptr[i]; j < this.R_Ptr[i + 1]; j++)
                {
                    ans[this.C_Index[j]] += this.Var[j] * V.Var[i];
                }
            }

            Vec V_ans = new Vec(ans);

            return V_ans;
        }


        public void Merge(CRS other)
        {
            if (this.Var.Count == 0)
            {
                this.Var = other.Var;
                this.C_Index = other.C_Index;
                this.R_Ptr = other.R_Ptr;
                this.C_Size = other.C_Size;
                this.R_Size = other.R_Size;
            }
            else
            {
                int c = this.Var.Count;
                this.Var.AddRange(other.Var);
                this.C_Index.AddRange(other.C_Index);
                foreach (int r in other.R_Ptr)
                {
                    if (r != 0)
                    {
                        this.R_Ptr.Add(r + c);
                    }
                }
                this.R_Size += other.R_Size;
            }

        }

        public static Vec operator *(CRS Mat, Vec V)
        {
            List<double> ans = new List<double>();

            for (int i = 0; i < Mat.R_Ptr.Count - 1; i++)
            {
                double a = 0;
                for (int j = Mat.R_Ptr[i]; j < Mat.R_Ptr[i + 1]; j++)
                {
                    a += Mat.Var[j] * V.Var[Mat.C_Index[j]];
                }
                ans.Add(a);
            }

            Vec V_ans = new Vec(ans);

            return V_ans;
        }

        public override string ToString()
        {
            string st = "";

            for (int i = 0; i < this.Var.Count; i++)
            {
                st += Var[i] + " ";
                st += C_Index[i] + " ";
                if (i < R_Ptr.Count)
                {
                    st += R_Ptr[i] + " ";
                }
                st += "\r\n";
            }

            return st;
        }

    }

    public class Vec
    {
        public List<double> Var;

        public Vec(List<double> _Var)
        {
            this.Var = new List<double>(_Var);
        }

        public Vec()
        {
            this.Var = new List<double>();
        }

        public Vec(Mesh m)
        {
            this.Var = new List<double>();
            foreach (Point3f v in m.Vertices)
            {
                this.Var.Add(v.X);
                this.Var.Add(v.Y);
                this.Var.Add(v.Z);
            }
        }

        public double NormSquared()
        {
            double ans = 0;

            foreach (double var in this.Var)
            {
                ans += var * var;
            }

            return ans;
        }

        public void Merge(Vec other)
        {
            this.Var.AddRange(other.Var);
        }

        public void normalize()
        {
            double norm = (double)Math.Sqrt(this.NormSquared());
            for (int i = 0; i < this.Var.Count; i++)
            {
                this.Var[i] /= norm;
            }
        }

        public override string ToString()
        {
            string str = "";
            foreach (double v in this.Var)
            {
                str += v;
                str += "\r\n";
            }
            return str;
        }

        public static Vec operator +(Vec left, Vec right)
        {

            List<double> ans = new List<double>();

            for (int i = 0; i < left.Var.Count; i++)
            {
                ans.Add(left.Var[i] + right.Var[i]);
            }

            Vec V_ans = new Vec(ans);
            return V_ans;

        }

        public static Vec operator -(Vec left, Vec right)
        {

            List<double> ans = new List<double>();

            for (int i = 0; i < left.Var.Count; i++)
            {
                ans.Add(left.Var[i] - right.Var[i]);
            }

            Vec V_ans = new Vec(ans);
            return V_ans;

        }

        public static Vec operator *(double left, Vec right)
        {
            List<double> ans = new List<double>();

            for (int i = 0; i < right.Var.Count; i++)
            {
                ans.Add(left * right.Var[i]);
            }

            Vec V_ans = new Vec(ans);
            return V_ans;
        }

        public static double operator *(Vec left, Vec right)
        {
            double ans = 0;

            for (int i = 0; i < right.Var.Count; i++)
            {
                ans += left.Var[i] * right.Var[i];
            }

            return ans;
        }

        public static Vec TransformFromDennseVector(DenseVector dv)
        {
            Vec vec = new Vec();
            vec.Var.AddRange(dv.ToArray());

            return vec;
        }


    }

    public abstract class Constraint : GH_Component
    {

        public Constraint(string _name, string _nickname, string _description, string _category, string _subCategory)
          : base(_name, _nickname, _description, _category, _subCategory)
        {
        }


        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
        }

        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        protected override void SolveInstance(IGH_DataAccess DA)
        {
        }

        public abstract CRS Jacobian(CMesh cm);

        public abstract Vec Error(CMesh cm);

        public abstract bool IsForRigidMode();

        protected override System.Drawing.Bitmap Icon
        {
            get
            {

                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return null;
            }
        }

        public abstract override Guid ComponentGuid
        {
            get;
        }
    }

    public class CMesh : Mesh
    {
        /// CMeshが持つメンバ
        /// mesh        :もとのメッシュ
        /// edgeInfo    : 
        ///      "M"    :Mountain
        ///      "V"    :Valley
        ///      "B"    :Boundary
        ///      "T"    :Tryangulated
        ///      'U'    :Unassigned 
        /// inner_edges ;内部辺のTopologyEdges
        /// face_pairs  :inner_edges の Connected Faces
        /// foldang     ;折角

        public Mesh mesh = new Mesh();
        public List<Char> edgeInfo;
        public MeshFaceList orig_faces;
        public List<Char> inner_edge_assignment;
        public List<double> foldang;

        public List<IndexPair> inner_edges;
        public List<IndexPair> boundary_edges;
        public List<int> inner_boundary_edges;
        public List<IndexPair> triangulated_edges;
        public List<IndexPair> mountain_edges;
        public List<IndexPair> valley_edges;

        public List<IndexPair> face_pairs;
        public List<IndexPair> triangulated_face_pairs;
        public List<IndexPair> mountain_face_pairs;
        public List<IndexPair> valley_face_pairs;

        public List<Tuple<double, double>> face_height_pairs;
        public List<Tuple<double, double>> triangulated_face_height_pairs;
        public List<Tuple<double, double>> mountain_face_height_pairs;
        public List<Tuple<double, double>> valley_face_height_pairs;

        public List<double> length_of_diagonal_edges;
        public List<double> length_of_triangulated_diagonal_edges;
        public List<double> length_of_mountain_diagonal_edges;
        public List<double> length_of_valley_diagonal_edges;
        public List<double> Initial_edges_length;

        public CMesh(Mesh _mesh, List<Char> _edgeinfo)
        {
            this.mesh = _mesh.DuplicateMesh();
            this.edgeInfo = _edgeinfo;
            this.orig_faces = _mesh.Faces;
        }

        public CMesh(Mesh _mesh)
        {
            this.mesh = _mesh.DuplicateMesh();
            this.orig_faces = _mesh.Faces;
            var tri = this.InsertTriangulate();
            var edges = this.mesh.TopologyEdges;

            List<bool> naked = new List<bool>(_mesh.GetNakedEdgePointStatus());

            this.edgeInfo = new List<Char>();
            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                if (edges.GetConnectedFaces(i).Count() == 1)
                {
                    edgeInfo.Add('B');
                }
                if (edges.IsNgonInterior(i))
                {
                    edgeInfo.Add('T');
                }
                else
                {
                    edgeInfo.Add('U');
                }
            }

            if (tri.Count != 0)
            {
                foreach (List<int> v in tri)
                {
                    int ind = edges.GetEdgeIndex(v[0], v[1]);
                    if (ind != -1)
                    {
                        this.edgeInfo[ind] = 'T';
                    }
                }
            }
            GetMeshFundamentalInfo();
            GetFacePairBasicInfo(this);
            GetTriangulatedFacePairBasicInfo(this);
            GetMountainFacePairBasicInfo(this);
            GetValleyFacePairBasicInfo(this);
            this.inner_edge_assignment = GetInnerEdgeAssignment();
        }

        public CMesh(CMesh _cmesh)
        {
            this.mesh = _cmesh.mesh.DuplicateMesh();
            this.edgeInfo = _cmesh.edgeInfo;
            this.orig_faces = _cmesh.orig_faces;
            this.inner_edges = _cmesh.inner_edges;
            this.inner_edge_assignment = _cmesh.inner_edge_assignment;
            this.boundary_edges = _cmesh.boundary_edges;
            this.inner_boundary_edges = _cmesh.inner_boundary_edges;
            this.face_pairs = _cmesh.face_pairs;
            this.face_height_pairs = _cmesh.face_height_pairs;
            this.length_of_diagonal_edges = _cmesh.length_of_diagonal_edges;
            this.foldang = _cmesh.foldang;
            this.triangulated_edges = _cmesh.triangulated_edges;
            this.triangulated_face_pairs = _cmesh.triangulated_face_pairs;
            this.triangulated_face_height_pairs = _cmesh.triangulated_face_height_pairs;
            this.length_of_triangulated_diagonal_edges = _cmesh.length_of_triangulated_diagonal_edges;
            this.mountain_edges = _cmesh.mountain_edges;
            this.mountain_face_pairs = _cmesh.mountain_face_pairs;
            this.mountain_face_height_pairs = _cmesh.mountain_face_height_pairs;
            this.length_of_mountain_diagonal_edges = _cmesh.length_of_mountain_diagonal_edges;
            this.valley_edges = _cmesh.valley_edges;
            this.valley_face_pairs = _cmesh.valley_face_pairs;
            this.valley_face_height_pairs = _cmesh.valley_face_height_pairs;
            this.length_of_valley_diagonal_edges = _cmesh.length_of_valley_diagonal_edges;
        }

        public CMesh(Mesh _mesh, List<Line> M, List<Line> V)
        {
            this.mesh = _mesh.DuplicateMesh();
            this.mesh.FaceNormals.ComputeFaceNormals();
            this.orig_faces = _mesh.Faces;
            var tri = this.InsertTriangulate();

            var edges = mesh.TopologyEdges;
            var verts = mesh.TopologyVertices;

            this.edgeInfo = new List<Char>();
            for (int i = 0; i < edges.Count; i++)
            {
                if (edges.IsNgonInterior(i))
                {
                    edgeInfo.Add('T');
                }
                else
                {
                    edgeInfo.Add('U');
                }
            }

            foreach (Line m in M)
            {
                Point3d a = m.From;
                Point3d b = m.To;

                int found = 0;
                int vertsCount = verts.Count;
                int j = 0;
                int fromIndex = -1;
                int toIndex = -1;
                while (found != 2 && j < vertsCount)
                {
                    Point3d p = verts[j];

                    if (p.DistanceTo(a) < 0.1)
                    {
                        fromIndex = j;
                        found++;
                    }
                    else if (p.DistanceTo(b) < 0.1)
                    {
                        toIndex = j;
                        found++;
                    }
                    j++;
                }
                int ind = edges.GetEdgeIndex(fromIndex, toIndex);
                if (ind != -1)
                {
                    edgeInfo[ind] = 'M';
                }
            }
            foreach (Line v in V)
            {
                Point3d a = v.From;
                Point3d b = v.To;

                int found = 0;
                int vertsCount = verts.Count;
                int j = 0;
                int fromIndex = -1;
                int toIndex = -1;
                while (found != 2 && j < vertsCount)
                {
                    Point3d p = verts[j];

                    if (p.DistanceTo(a) < 0.1)
                    {
                        fromIndex = j;
                        found++;
                    }
                    else if (p.DistanceTo(b) < 0.1)
                    {
                        toIndex = j;
                        found++;
                    }
                    j++;
                }
                int ind = edges.GetEdgeIndex(fromIndex, toIndex);
                if (ind != -1)
                {
                    edgeInfo[ind] = 'V';
                }
            }
            if (tri.Count != 0)
            {
                foreach (List<int> v in tri)
                {
                    int ind = edges.GetEdgeIndex(v[0], v[1]);
                    if (ind != -1)
                    {
                        this.edgeInfo[ind] = 'T';
                    }
                }
            }

            List<bool> naked = new List<bool>(_mesh.GetNakedEdgePointStatus());

            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                if (edges.GetConnectedFaces(i).Count() == 1)
                {

                    this.edgeInfo[i] = 'B';
                }
            }
            GetMeshFundamentalInfo();
            GetTriangulatedFacePairBasicInfo(this);
            GetMountainFacePairBasicInfo(this);
            GetValleyFacePairBasicInfo(this);
            this.inner_edge_assignment = GetInnerEdgeAssignment();
        }

        public CMesh(Mesh _mesh, List<Line> M, List<Line> V, List<Line> T)
        {
            this.mesh = _mesh.DuplicateMesh();
            this.mesh.FaceNormals.ComputeFaceNormals();
            this.orig_faces = _mesh.Faces;
            var tri = this.InsertTriangulate();

            var edges = mesh.TopologyEdges;
            var verts = mesh.TopologyVertices;

            this.edgeInfo = new List<Char>();
            for (int i = 0; i < edges.Count; i++)
            {
                if (edges.IsNgonInterior(i))
                {
                    edgeInfo.Add('T');
                }
                else
                {
                    edgeInfo.Add('U');
                }
            }

            foreach (Line m in M)
            {
                Point3d a = m.From;
                Point3d b = m.To;

                int found = 0;
                int vertsCount = verts.Count;
                int j = 0;
                int fromIndex = -1;
                int toIndex = -1;
                while (found != 2 && j < vertsCount)
                {
                    Point3d p = verts[j];

                    if (p.DistanceTo(a) < 0.1)
                    {
                        fromIndex = j;
                        found++;
                    }
                    else if (p.DistanceTo(b) < 0.1)
                    {
                        toIndex = j;
                        found++;
                    }
                    j++;
                }
                int ind = edges.GetEdgeIndex(fromIndex, toIndex);
                if (ind != -1)
                {
                    edgeInfo[ind] = 'M';
                }
            }
            foreach (Line v in V)
            {
                Point3d a = v.From;
                Point3d b = v.To;

                int found = 0;
                int vertsCount = verts.Count;
                int j = 0;
                int fromIndex = -1;
                int toIndex = -1;
                while (found != 2 && j < vertsCount)
                {
                    Point3d p = verts[j];

                    if (p.DistanceTo(a) < 0.1)
                    {
                        fromIndex = j;
                        found++;
                    }
                    else if (p.DistanceTo(b) < 0.1)
                    {
                        toIndex = j;
                        found++;
                    }
                    j++;
                }
                int ind = edges.GetEdgeIndex(fromIndex, toIndex);
                if (ind != -1)
                {
                    edgeInfo[ind] = 'V';
                }
            }
            if (tri.Count != 0)
            {
                foreach (List<int> v in tri)
                {
                    int ind = edges.GetEdgeIndex(v[0], v[1]);
                    if (ind != -1)
                    {
                        this.edgeInfo[ind] = 'T';
                    }
                }
            }

            foreach (Line t in T)
            {
                Point3d a = t.From;
                Point3d b = t.To;

                int found = 0;
                int vertsCount = verts.Count;
                int j = 0;
                int fromIndex = -1;
                int toIndex = -1;
                while (found != 2 && j < vertsCount)
                {
                    Point3d p = verts[j];

                    if (p.DistanceTo(a) < 0.1)
                    {
                        fromIndex = j;
                        found++;
                    }
                    else if (p.DistanceTo(b) < 0.1)
                    {
                        toIndex = j;
                        found++;
                    }
                    j++;
                }
                int ind = edges.GetEdgeIndex(fromIndex, toIndex);
                if (ind != -1)
                {
                    edgeInfo[ind] = 'T';
                }
            }

            List<bool> naked = new List<bool>(_mesh.GetNakedEdgePointStatus());

            for (int i = 0; i < mesh.TopologyEdges.Count; i++)
            {
                if (edges.GetConnectedFaces(i).Count() == 1)
                {

                    this.edgeInfo[i] = 'B';
                }
            }
            GetMeshFundamentalInfo();
            GetTriangulatedFacePairBasicInfo(this);
            GetMountainFacePairBasicInfo(this);
            GetValleyFacePairBasicInfo(this);
            this.inner_edge_assignment = GetInnerEdgeAssignment();
        }

        public CMesh(Mesh _mesh, List<List<int>> M_indexPairs, List<List<int>> V_indexPairs) : this(_mesh, PairsToLines(_mesh, M_indexPairs), PairsToLines(_mesh, V_indexPairs))
        {
        }

        static List<Line> PairsToLines(Mesh mesh, List<List<int>> Pairs)
        {
            var verts = mesh.Vertices;

            List<Line> M = new List<Line>();
            foreach (var m in Pairs)
            {
                M.Add(new Line(verts[m[0]], verts[m[1]]));
            }

            return M;

        }

        private List<List<int>> InsertTriangulate()
        {
            var faces = this.mesh.Faces;
            var verts = this.mesh.Vertices;

            List<List<int>> ans = new List<List<int>>();

            List<MeshFace> newFaces = new List<MeshFace>();

            for (int i = 0; i < faces.Count; i++)
            {
                MeshFace face = faces[i];
                if (face[2] != face[3])
                {
                    double d1 = verts[face[0]].DistanceTo(verts[face[2]]);
                    double d2 = verts[face[1]].DistanceTo(verts[face[3]]);
                    if (d1 >= d2)
                    {
                        newFaces.Add(new MeshFace(face[1], face[2], face[0]));
                        newFaces.Add(new MeshFace(face[3], face[0], face[2]));
                        List<int> temp = new List<int>();
                        temp.Add(face[0]);
                        temp.Add(face[2]);
                        ans.Add(temp);
                    }
                    else
                    {
                        newFaces.Add(new MeshFace(face[0], face[1], face[3]));
                        newFaces.Add(new MeshFace(face[2], face[3], face[1]));
                        List<int> temp = new List<int>();
                        temp.Add(face[1]);
                        temp.Add(face[3]);
                        ans.Add(temp);
                    }
                }
                else
                {
                    newFaces.Add(new MeshFace(face[0], face[1], face[2]));
                }
            }

            this.mesh.Faces.Destroy();
            this.mesh.Faces.AddFaces(newFaces);

            return ans;
        }

        public List<double> GetFoldAngle()
        {
            var edges = this.inner_edges;
            var verts = this.mesh.Vertices;
            var faces = this.face_pairs;

            List<double> foldang = new List<double>();

            this.mesh.FaceNormals.ComputeFaceNormals();

            for (int e = 0; e < edges.Count; e++)
            {
                double foldang_e = 0;
                int u = edges[e].I;
                int v = edges[e].J;
                int p = faces[e].I;
                int q = faces[e].J;
                Vector3d normal_i = this.mesh.FaceNormals[p];
                Vector3d normal_j = this.mesh.FaceNormals[q];
                /// cos(foldang_e) = n_i * n_j
                double cos_foldang_e = normal_i * normal_j;
                /// sin(foldang_e) = n_i × n_j
                Vector3d vec_e = verts[u] - verts[v];
                vec_e.Unitize();
                double sin_foldang_e = Vector3d.CrossProduct(normal_i, normal_j) * vec_e;
                if (sin_foldang_e >= 0)
                {
                    if (cos_foldang_e >= 1.0)
                    {
                        foldang_e = 0;
                    }
                    else if (cos_foldang_e <= -1.0)
                    {
                        foldang_e = Math.PI;
                    }
                    else
                    {
                        foldang_e = Math.Acos(cos_foldang_e);
                    }
                }
                else
                {
                    if (cos_foldang_e >= 1.0)
                    {
                        foldang_e = 0;
                    }
                    else if (cos_foldang_e <= -1.0)
                    {
                        foldang_e = -Math.PI;
                    }
                    else
                    {
                        foldang_e = -Math.Acos(cos_foldang_e);
                    }
                }
                foldang.Add(foldang_e);
            }
            return foldang;
        }

        public List<Char> GetInnerEdgeAssignment()
        {
            List<Char> inner_edge_info = new List<Char>();
            for (int i = 0; i < this.edgeInfo.Count; i++)
            {
                if (edgeInfo[i] != 'B')
                {
                    inner_edge_info.Add(edgeInfo[i]);
                }
            }
            return inner_edge_info;
        }

        public List<Char> GetTrianglatedEdgeAssignment()
        {
            List<Char> trianglated_edge_info = new List<Char>();
            for (int i = 0; i < this.edgeInfo.Count; i++)
            {
                if (edgeInfo[i] == 'T')
                {
                    trianglated_edge_info.Add(edgeInfo[i]);
                }
            }
            return trianglated_edge_info;
        }

        public void GetMeshFundamentalInfo()
        {
            this.inner_edges = new List<IndexPair>();
            this.boundary_edges = new List<IndexPair>();
            this.inner_boundary_edges = new List<int>();
            this.face_pairs = new List<IndexPair>();
            this.foldang = new List<double>();
            this.triangulated_edges = new List<IndexPair>();
            this.triangulated_face_pairs = new List<IndexPair>();
            this.mountain_edges = new List<IndexPair>();
            this.mountain_face_pairs = new List<IndexPair>();
            this.valley_edges = new List<IndexPair>();
            this.valley_face_pairs = new List<IndexPair>();
            List<int> inner_edges_id = new List<int>();
            List<int> boundary_edges_id = new List<int>();

            for (int e = 0; e < this.mesh.TopologyEdges.Count; e++)
            {
                if (this.mesh.TopologyEdges.GetConnectedFaces(e).Count() > 1)
                {
                    IndexPair inner_edge = new IndexPair();
                    inner_edge.I = this.mesh.TopologyEdges.GetTopologyVertices(e).I;
                    inner_edge.J = this.mesh.TopologyEdges.GetTopologyVertices(e).J;
                    this.inner_edges.Add(inner_edge);
                    inner_edges_id.Add(e);

                    IndexPair face_pair = GetFacePair(inner_edge, e);
                    this.face_pairs.Add(face_pair);
                }
                if (this.mesh.TopologyEdges.GetConnectedFaces(e).Count() == 1)
                {
                    IndexPair boundary_edge = new IndexPair();
                    boundary_edge.I = this.mesh.TopologyEdges.GetTopologyVertices(e).I;
                    boundary_edge.J = this.mesh.TopologyEdges.GetTopologyVertices(e).J;
                    this.boundary_edges.Add(boundary_edge);
                    boundary_edges_id.Add(e);
                }
                if (this.edgeInfo[e] == 'T')
                {
                    IndexPair triangulated_edge = new IndexPair();
                    triangulated_edge.I = this.mesh.TopologyEdges.GetTopologyVertices(e).I;
                    triangulated_edge.J = this.mesh.TopologyEdges.GetTopologyVertices(e).J;
                    this.triangulated_edges.Add(triangulated_edge);

                    IndexPair triangulated_face_pair = GetFacePair(triangulated_edge, e);
                    this.triangulated_face_pairs.Add(triangulated_face_pair);
                }
                if (this.edgeInfo[e] == 'M')
                {
                    IndexPair mountain_edge = new IndexPair();
                    mountain_edge.I = this.mesh.TopologyEdges.GetTopologyVertices(e).I;
                    mountain_edge.J = this.mesh.TopologyEdges.GetTopologyVertices(e).J;
                    this.mountain_edges.Add(mountain_edge);

                    IndexPair mountain_face_pair = GetFacePair(mountain_edge, e);
                    this.mountain_face_pairs.Add(mountain_face_pair);
                }
                if (this.edgeInfo[e] == 'V')
                {
                    IndexPair valley_edge = new IndexPair();
                    valley_edge.I = this.mesh.TopologyEdges.GetTopologyVertices(e).I;
                    valley_edge.J = this.mesh.TopologyEdges.GetTopologyVertices(e).J;
                    this.valley_edges.Add(valley_edge);

                    IndexPair valley_face_pair = GetFacePair(valley_edge, e);
                    this.valley_face_pairs.Add(valley_face_pair);
                }
            }
            this.inner_boundary_edges.AddRange(inner_edges_id);
            this.inner_boundary_edges.AddRange(boundary_edges_id);
            this.foldang = GetFoldAngle();
        }

        public void GetFacePairBasicInfo(CMesh cm)
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

        public void GetTriangulatedFacePairBasicInfo(CMesh cm)
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

        public void GetMountainFacePairBasicInfo(CMesh cm)
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

        public void GetValleyFacePairBasicInfo(CMesh cm)
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

        private Boolean SortFacePair(MeshFace f, IndexPair e)
        {
            IndexPair f1 = new IndexPair(f.A, f.B);
            IndexPair f2 = new IndexPair(f.B, f.C);
            IndexPair f3 = new IndexPair(f.C, f.A);
            Boolean Bool = false;
            Boolean b1 = (f1.I == e.I) & (f1.J == e.J);
            Boolean b2 = (f2.I == e.I) & (f2.J == e.J);
            Boolean b3 = (f3.I == e.I) & (f3.J == e.J);
            Bool = b1 | b2 | b3;

            return Bool;
        }

        private IndexPair GetFacePair(IndexPair edge_pair, int e)
        {
            int f_ind_0 = this.mesh.TopologyEdges.GetConnectedFaces(e)[0];
            int f_ind_1 = this.mesh.TopologyEdges.GetConnectedFaces(e)[1];
            MeshFace f = this.mesh.Faces[f_ind_0];
            IndexPair face_pair;
            if (SortFacePair(f, edge_pair))
            {
                face_pair = new IndexPair(f_ind_0, f_ind_1);
            }
            else
            {
                face_pair = new IndexPair(f_ind_1, f_ind_0);
            }

            return face_pair;

        }
    }
}

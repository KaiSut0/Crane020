using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class FlatFoldable : Constraint
    {
        /// <summary>
        /// Initializes a new instance of the FlatFoldable class.
        /// </summary>
        public FlatFoldable()
          : base("FlatFoldable", "FlatFoldable",
              "Add FlatFoldability",
              "Crane", "Constraints")
        {
        }

        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Constraint", "C", "Constraint", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            DA.SetData(0, this);
        }

        public override CRS Jacobian(CMesh cm)
        {
            Mesh m = cm.mesh;

            List<double> var = new List<double>();
            List<int> r_index = new List<int>();
            List<int> c_index = new List<int>();

            var topo = m.TopologyVertices;
            topo.SortEdges();

            List<bool> isNaked = new List<bool>(m.GetNakedEdgePointStatus());
            List<int> internalVertices = new List<int>();

            //内部頂点インデックスリスト作成
            for (int i = 0; i < topo.Count; i++)
            {
                if (!isNaked[i])
                {
                    internalVertices.Add(i);
                }
            }

            //各内部頂点について
            for (int r = 0; r < internalVertices.Count; r++)
            {
                //内部頂点のインデックス、位置ベクトル
                int index_center = internalVertices[r];
                Vector3d center = new Vector3d(topo[index_center]);

                //接続点のインデックス、位置ベクトル
                List<int> index_neighbors = new List<int>();
                List<Vector3d> neighbors = new List<Vector3d>();
                index_neighbors.AddRange(topo.ConnectedTopologyVertices(index_center));

                //エッジベクトル
                List<Vector3d> vecs = new List<Vector3d>();
                List<Vector3d> normals = new List<Vector3d>();

                int n = index_neighbors.Count;

                //三角形分割の区別
                List<int> connected_edges = new List<int>(topo.ConnectedEdges(index_center));
                List<double> signs = new List<double>();
                for (int i = -1; i < n-1; i++)
                {
                    if (i == -1)
                    {
                        signs.Add(1);
                    }
                    else if (cm.edgeInfo[connected_edges[i]]!='T')
                    {
                        signs.Add(-signs[i]);
                    }
                    else
                    {
                        signs.Add(signs[i]);
                    }
                }

                //位置ベクトル取得
                foreach (int index_neighbor in index_neighbors)
                {
                    neighbors.Add(new Vector3d(topo[index_neighbor]));
                }

                //方向ベクトル取得
                foreach (Vector3d neighbor in neighbors)
                {
                    Vector3d temp = neighbor - center;
                    vecs.Add(temp);
                }

                //法線ベクトル取得
                for (int i = 0; i < n; i++)
                {
                    Vector3d temp = new Vector3d();
                    if (i == 0)
                    {
                        temp = Vector3d.CrossProduct(vecs[n - 1], vecs[0]);
                    }
                    else
                    {
                        temp = Vector3d.CrossProduct(vecs[i - 1], vecs[i]);
                    }
                    temp.Unitize();
                    normals.Add(temp);
                }

                Vector3d v_center = new Vector3d();

                for (int i = 0; i < n; i++)
                {
                    int index = index_neighbors[i];
                    Vector3d v1 = signs[i] * Vector3d.CrossProduct(normals[i], vecs[i]);
                    v1 /= vecs[i].SquareLength;
                    Vector3d v2 = signs[(i + 1) % n] * Vector3d.CrossProduct(normals[(i + 1) % n], vecs[i]);
                    v2 /= vecs[i].SquareLength;
                    v1 -= v2;
                    v_center -= v1;


                    var.Add((double)v1.X);
                    var.Add((double)v1.Y);
                    var.Add((double)v1.Z);
                    c_index.Add(index * 3);
                    c_index.Add(index * 3 + 1);
                    c_index.Add(index * 3 + 2);
                    r_index.Add(r);
                    r_index.Add(r);
                    r_index.Add(r);
                }
                var.Add((double)v_center.X);
                var.Add((double)v_center.Y);
                var.Add((double)v_center.Z);
                c_index.Add(index_center * 3);
                c_index.Add(index_center * 3 + 1);
                c_index.Add(index_center * 3 + 2);
                r_index.Add(r);
                r_index.Add(r);
                r_index.Add(r);

            }

            CRS Jaco = new CRS(var, r_index, c_index, internalVertices.Count, topo.Count * 3);

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            Mesh m = cm.mesh;

            List<double> err = new List<double>();

            m.Normals.ComputeNormals();
            var topo = m.TopologyVertices;
            topo.SortEdges();

            List<bool> isNaked = new List<bool>(m.GetNakedEdgePointStatus());
            List<int> internalVertices = new List<int>();
            for (int i = 0; i < topo.Count; i++)
            {
                if (!isNaked[i])
                {
                    internalVertices.Add(i);
                }
            }

            //各内部頂点について
            for (int r = 0; r < internalVertices.Count; r++)
            {
                //内部頂点のインデックス、位置ベクトル
                int index_center = internalVertices[r];
                Vector3d center = new Vector3d(topo[index_center]);

                //接続点のインデックス、位置ベクトル
                List<int> index_neighbors = new List<int>();
                List<Vector3d> neighbors = new List<Vector3d>();
                index_neighbors.AddRange(topo.ConnectedTopologyVertices(index_center));

                //エッジベクトル
                List<Vector3d> vecs = new List<Vector3d>();
                double sum = 0;

                int n = index_neighbors.Count;

                //三角形分割の区別
                List<int> connected_edges = new List<int>(topo.ConnectedEdges(index_center));
                List<bool> is_corrects = new List<bool>();

                for (int i = 0; i < n; i++)
                {
                    bool is_correct = m.TopologyEdges.GetTopologyVertices(connected_edges[i]).Contains(index_neighbors[i]);
                    is_corrects.Add(is_correct);
                }

                List<double> signs = new List<double>();
                for (int i = -1; i < n - 1; i++)
                {
                    if (i == -1)
                    {
                        signs.Add(1);
                    }
                    else if (cm.edgeInfo[connected_edges[i]] != 'T')
                    {
                        signs.Add(-signs[i]);
                    }
                    else
                    {
                        signs.Add(signs[i]);
                    }
                }

                //位置ベクトル取得
                foreach (int index_neighbor in index_neighbors)
                {
                    neighbors.Add(new Vector3d(topo[index_neighbor]));
                }

                //方向ベクトル取得
                foreach (Vector3d neighbor in neighbors)
                {
                    Vector3d temp = neighbor - center;
                    vecs.Add(temp);
                }

                for (int i = 0; i < n; i++)
                {
                    if (i == 0)
                    {
                        sum += signs[i] * Vector3d.VectorAngle(vecs[n - 1], vecs[0]);
                    }
                    else
                    {
                        sum += signs[i] * Vector3d.VectorAngle(vecs[i - 1], vecs[i]);
                    }
                }

                err.Add((double)sum);
            }

            Vec ans = new Vec(err);

            return ans;

        }

        public override bool IsForRigidMode()
        {
            return false;
        }
        /// <summary>
        /// Provides an Icon for every component that will be visible in the User Interface.
        /// Icons need to be 24x24 pixels.
        /// </summary>
        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Crane.Properties.Resources.flat_foldable;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("5914c562-9b83-4da8-b219-48d23bfbb575"); }
        }
    }
}
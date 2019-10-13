using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Collections;

namespace Crane
{
    public class EqualLength : Constraint
    {
        List<int> equal_edge_ids;
        double edge_length;
        bool is_set_length;
        double edge_avarage_length;
        /// <summary>
        /// Initializes a new instance of the EqualLength class.
        /// </summary>
        public EqualLength()
          : base("EqualLength", "EqualLength",
              "Equal length constraint between selected edges. You can also set length of selected edges",
              "Crane", "Constraints")
        {
            equal_edge_ids = new List<int>();
            edge_length = 0;
            is_set_length = false;
            edge_avarage_length = 0;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "Input CMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("EdgeID", "EdgeID", "Input Indices of edge for equal length constraint", GH_ParamAccess.list);
            pManager.AddNumberParameter("SetLength", "SetLength", "", GH_ParamAccess.item);

            pManager[2].Optional = true;
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
            equal_edge_ids = new List<int>();
            is_set_length = false;
            CMesh cmesh = null;
            if (!DA.GetData(0, ref cmesh)) { return; }
            if (!DA.GetDataList(1, equal_edge_ids)) { return; }
            is_set_length = DA.GetData(2, ref edge_length);

            int edge_count = cmesh.mesh.TopologyEdges.Count;
            for (int i = 0; i < edge_count; i++)
            {
                edge_avarage_length += cmesh.mesh.TopologyEdges.EdgeLine(i).Length;
            }
            edge_avarage_length /= edge_count;

            DA.SetData(0, this);
        }

        /// <summary>
        /// Provides an Icon for the component.
        /// </summary>

        public override CRS Jacobian(CMesh cm)
        {
            Mesh m = cm.mesh;
            List<double> var = new List<double>();
            List<int> r_index = new List<int>();
            List<int> c_index = new List<int>();
            CRS Jaco = new CRS(var, r_index, c_index, 0, 0);

            List<Point3d> verts = new List<Point3d>(m.Vertices.ToPoint3dArray());

            // 等長拘束をかける辺の数取得
            int n = equal_edge_ids.Count;

            // メッシュの TopologyEdges 取得
            MeshTopologyEdgeList topo_edge = m.TopologyEdges;
            List<int> edges_id = cm.inner_boundary_edges;


            // 辺長一定拘束 On の時
            if (is_set_length)
            {
                for (int i = 0; i < n; i++)
                {
                    int edge_id = edges_id[equal_edge_ids[i]];
                    int st_pti_id = topo_edge.GetTopologyVertices(edge_id).I;
                    int end_pti_id = topo_edge.GetTopologyVertices(edge_id).J;

                    Vector3d vsti = verts[st_pti_id] - verts[end_pti_id];
                    Vector3d veti = -vsti;

                    for (int j = 0; j < 3; j++)
                    {
                        var.Add(vsti[j]/(edge_avarage_length*edge_avarage_length));
                        r_index.Add(i);
                        c_index.Add(3 * st_pti_id + j);
                        var.Add(veti[j]/(edge_avarage_length*edge_avarage_length));
                        r_index.Add(i);
                        c_index.Add(3 * end_pti_id + j);
                    }
                }
                Jaco = new CRS(var, r_index, c_index, n, verts.Count * 3);
            }
            else
            {
                for (int i = 0; i < n - 1; i++)
                {
                    int edge_id_i = edges_id[equal_edge_ids[i]];
                    int edge_id_i1 = edges_id[equal_edge_ids[i+1]];

                    int st_pti_id = topo_edge.GetTopologyVertices(edge_id_i).I;
                    int end_pti_id = topo_edge.GetTopologyVertices(edge_id_i).J;
                    int st_pti1_id = topo_edge.GetTopologyVertices(edge_id_i1).I;
                    int end_pti1_id = topo_edge.GetTopologyVertices(edge_id_i1).J;

                    Vector3d vsti = verts[st_pti_id] - verts[end_pti_id];
                    Vector3d veti = -vsti;
                    Vector3d vsti1 = verts[end_pti1_id] - verts[st_pti1_id];
                    Vector3d veti1 = -vsti1;

                    for (int j = 0; j < 3; j++)
                    {
                        var.Add(vsti[j]/(edge_avarage_length*edge_avarage_length));
                        r_index.Add(i);
                        c_index.Add(3 * st_pti_id + j);
                        var.Add(veti[j]/(edge_avarage_length*edge_avarage_length));
                        r_index.Add(i);
                        c_index.Add(3 * end_pti_id + j);
                        var.Add(vsti1[j]/(edge_avarage_length*edge_avarage_length));
                        r_index.Add(i);
                        c_index.Add(3 * st_pti1_id + j);
                        var.Add(veti1[j]/(edge_avarage_length*edge_avarage_length));
                        r_index.Add(i);
                        c_index.Add(3 * end_pti1_id + j);
                    }

                }

                Jaco = new CRS(var, r_index, c_index, n-1, verts.Count * 3);
            }

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            List<double> err = new List<double>();
            Vec ans = new Vec(err);

            // 等長拘束をかける辺の数取得
            int n = equal_edge_ids.Count;

            // メッシュの TopologyEdges 取得
            MeshTopologyEdgeList topo_edge = cm.mesh.TopologyEdges;
            List<int> edges_id = cm.inner_boundary_edges;

            // 辺長一定拘束 On の時
            if (is_set_length)
            {
                for (int i = 0; i < n; i++)
                {
                    int edge_id = edges_id[equal_edge_ids[i]];
                    double edge_length_i = topo_edge.EdgeLine(edge_id).Length;
                    err.Add((edge_length_i * edge_length_i - edge_length * edge_length) / (2*edge_avarage_length*edge_avarage_length));
                }
                ans = new Vec(err);
            }

            // 辺長一定拘束 Off の時
            else
            {
                // 等長拘束の誤差取得
                for (int i = 0; i < n - 1; i++)
                {
                    int edge_id_i = edges_id[equal_edge_ids[i]];
                    int edge_id_i1 = edges_id[equal_edge_ids[i + 1]];

                    double edge_length_i = topo_edge.EdgeLine(edge_id_i).Length;
                    double edge_length_i1 = topo_edge.EdgeLine(edge_id_i1).Length;
                    err.Add((edge_length_i * edge_length_i - edge_length_i1 * edge_length_i1) / (2*edge_avarage_length*edge_avarage_length));
                }

                ans = new Vec(err);
            }


            return ans;
        }

        public override bool IsForRigidMode()
        {
            return false;
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                //You can add image files to your project resources and access them like this:
                // return Resources.IconForThisComponent;
                return Crane.Properties.Resources.equal_length;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("e3b80731-ea16-4fb7-9e1b-06e61a036c22"); }
        }
    }
}
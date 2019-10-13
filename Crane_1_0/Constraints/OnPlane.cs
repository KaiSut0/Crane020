using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class OnPlane : Constraint
    {
        // member object
        Plane goal_plane;
        List<int> anchor_vertex_ids = new List<int>();
        public double edge_avarage_length = 0;
        public double strength = 1.0;

        /// <summary>
        /// Initializes a new instance of the MyComponent1 class.
        /// </summary>
        public OnPlane()
          : base("OnPlane", "OnPlane",
              "On plane constraint of selected vertices",
              "Crane", "Constraints")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "Input CMesh", GH_ParamAccess.item);
            pManager.AddPlaneParameter("Plane", "P", "Input plane as a goal object", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Vertex Indices", "VertIDs", "Input vertex indices to anchor.", GH_ParamAccess.list);
            pManager.AddNumberParameter("Strength", "S", "Strength", GH_ParamAccess.item);

            pManager[3].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Constraint", "C", "On Plane Constraint", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 値をセットするごとに public member をリフレッシュ
            goal_plane = new Plane();
            anchor_vertex_ids = new List<int>();
            CMesh cm = null;
            strength = 1.0;

            // 値をセット
            if (!DA.GetData(0, ref cm)) { return; }
            if (!DA.GetData(1, ref goal_plane)) { return; }
            if (!DA.GetDataList(2, anchor_vertex_ids)) { return; }
            DA.GetData(3, ref strength);

            int edge_count = cm.mesh.TopologyEdges.Count;
            for (int i=0; i<edge_count; i++)
            {
                edge_avarage_length += cm.mesh.TopologyEdges.EdgeLine(i).Length;
            }
            edge_avarage_length /= edge_count;

            DA.SetData(0, this);
        }

        public override CRS Jacobian(CMesh cm)
        {
            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());

            List<double> var = new List<double>();
            List<int> r_index = new List<int>();
            List<int> c_index = new List<int>();

            for (int i = 0; i < anchor_vertex_ids.Count; i++)
            {
                Point3d pt_on_plane = goal_plane.ClosestPoint(verts[anchor_vertex_ids[i]]);

                for (int j = 0; j < 3; j++)
                {
                    var.Add(strength * (verts[anchor_vertex_ids[i]][j] - pt_on_plane[j]) / (edge_avarage_length*edge_avarage_length));
                    r_index.Add(i);
                    c_index.Add(3 * anchor_vertex_ids[i] + j);
                }
            }


            CRS Jaco = new CRS(var, r_index, c_index, anchor_vertex_ids.Count, verts.Count * 3);

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());

            List<double> err = new List<double>();

            foreach (int i in anchor_vertex_ids)
            {
                double dist = goal_plane.DistanceTo(verts[i]);
                err.Add(strength * dist * dist / (2 * edge_avarage_length*edge_avarage_length));
            }


            Vec ans = new Vec(err);

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
                return Crane.Properties.Resources.on_plane;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("1c1c29fd-cb5b-47e7-863d-1f27ada2f9d9"); }
        }
    }
}
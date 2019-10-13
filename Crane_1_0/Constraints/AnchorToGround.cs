using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;


namespace Crane
{
    public class AnchorToGround : Constraint
    {
        List<int> anchors_index = new List<int>();
        public double edge_avarage_length = 0;
        public double strength = 1.0;
        /// <summary>
        /// Initializes a new instance of the AnchorToGround class.
        /// </summary>
        public AnchorToGround()
          : base("AnchorToGround", "AnchorToGround",
              "Anchor to ground constraint of selected vertices",
              "Crane", "Constraints")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "Input CMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("VerticesIndex", "VertInd", "Indexes of vertex to anchor to ground", GH_ParamAccess.list);
            pManager.AddNumberParameter("Strength", "S", "Strength", GH_ParamAccess.item);

            pManager[2].Optional = true;
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
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            CMesh cm = null;
            anchors_index = new List<int>();
            strength = 1.0;

            if (!DA.GetData(0, ref cm)) { return; }
            if (!DA.GetDataList(1, anchors_index)) { return; }
            DA.GetData(2, ref strength);

            int edge_count = cm.mesh.TopologyEdges.Count;
            for (int i = 0; i < edge_count; i++)
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

            for (int i = 0; i < this.anchors_index.Count; i++)
            {
                double z = verts[anchors_index[i]].Z;
                var.Add(z/(strength * edge_avarage_length*edge_avarage_length));
                r_index.Add(i);
                c_index.Add(3 * this.anchors_index[i] + 2);
            }


            CRS Jaco = new CRS(var, r_index, c_index, this.anchors_index.Count, verts.Count * 3);

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());

            List<double> err = new List<double>();

            foreach (int i in this.anchors_index)
            {
                double z = verts[i].Z;
                err.Add(z*z/(strength * 2*edge_avarage_length*edge_avarage_length));
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
                return Crane.Properties.Resources.anchor_to_ground;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("9cde2aa0-e33e-4e5f-8131-d37e2de81ab8"); }
        }
    }
}
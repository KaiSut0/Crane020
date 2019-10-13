using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class OnCurve : Constraint
    {

        List<int> anchors_index = new List<int>();
        Curve anchor_curve;
        public double edge_avarage_length = 0;
        public double strength = 1.0;
        /// <summary>
        /// Initializes a new instance of the OnCurve class.
        /// </summary>
        public OnCurve()
          : base("OnCurve", "OnCurve",
              "OnCurveConstraint",
              "Crane", "Constraints")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "Input CMesh", GH_ParamAccess.item);
            pManager.AddCurveParameter("Curve", "Curve", "Input curve as a goal object", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Indexes of Point", "PtId", "Input vertex indices to anchor", GH_ParamAccess.list);
            pManager.AddNumberParameter("Strength", "S", "Strength", GH_ParamAccess.item);

            pManager[3].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("Constraint", "Constraint", "Constraint", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 値をセットするごとに public member をリフレッシュ
            anchors_index = new List<int>();
            CMesh cm = null;
            strength = 1.0;

            if (!DA.GetData(0, ref cm)) { return; }
            if (!DA.GetData(1, ref anchor_curve)) { return; }
            if (!DA.GetDataList(2, anchors_index)) { return; }
            DA.GetData(3, ref strength);

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

            for (int i = 0; i < anchors_index.Count; i++)
            {
                double parm;
                anchor_curve.ClosestPoint(verts[anchors_index[i]], out parm);
                Point3d pt_on_curve = anchor_curve.PointAt(parm);

                for (int j = 0; j < 3; j++)
                {
                    var.Add(strength * (verts[anchors_index[i]][j] - pt_on_curve[j]) / (edge_avarage_length*edge_avarage_length));
                    r_index.Add(i);
                    c_index.Add(3 * anchors_index[i] + j);
                }
            }


            CRS Jaco = new CRS(var, r_index, c_index, anchors_index.Count, verts.Count * 3);

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());

            List<double> err = new List<double>();



            foreach (int i in anchors_index)
            {
                double parm;
                anchor_curve.ClosestPoint(verts[i], out parm);
                double dist = anchor_curve.PointAt(parm).DistanceTo(verts[i]);
                err.Add(strength * dist * dist / (2*edge_avarage_length*edge_avarage_length));
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
                return Crane.Properties.Resources.on_curve;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("195b1ffb-70aa-4ff1-9721-12b612145dd7"); }
        }
    }
}
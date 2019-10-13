using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class Anchor : Constraint
    {

        List<Point3d> anchors_position = new List<Point3d>();
        List<int> anchors_index = new List<int>();
        public double edge_avarage_length = 0;
        public double strength = 0;

        /// <summary>
        /// Initializes a new instance of the Anchor class.
        /// </summary>
        public Anchor()
          : base("Anchor", "Anchor",
              "Anchor constraints of between selected vertices and input points. Numbers of both list must be equal.",
              "Crane", "Constraints")
        {
            anchors_position = new List<Point3d>();
            anchors_index = new List<int>();
            strength = 0;
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "CMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Indexies of Point", "Indexies of Point", "Input list of index of point", GH_ParamAccess.tree);
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
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 値をセットする毎に public member をリフレッシュ
            anchors_position = new List<Point3d>();
            anchors_index = new List<int>();
            strength = 1.0;

            
            CMesh cmesh = null;

            if (!DA.GetData(0, ref cmesh)) { return; }
            if (!DA.GetDataList(1, anchors_index)) { return; }
            DA.GetData(2, ref strength);


            anchors_position = new List<Point3d>(cmesh.mesh.Vertices.ToPoint3dArray());
            int edge_count = cmesh.mesh.TopologyEdges.Count;
            for (int i = 0; i < edge_count; i++)
            {
                edge_avarage_length += cmesh.mesh.TopologyEdges.EdgeLine(i).Length;
            }
            edge_avarage_length /= edge_count;

            DA.SetData(0, this);
        }

        public override CRS Jacobian(CMesh cm)
        {
            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());
            Mesh m = cm.mesh;


            List<double> var = new List<double>();
            List<int> r_index = new List<int>();
            List<int> c_index = new List<int>();

            for (int i = 0; i < this.anchors_index.Count; i++)
            {
                Point3d ancpt = this.anchors_position[this.anchors_index[i]];
                Point3d pt = m.Vertices[this.anchors_index[i]];
                for (int j=0; j < 3; j++)
                {
                    var.Add(strength * (pt[j] - ancpt[j])/(edge_avarage_length*edge_avarage_length));
                    r_index.Add(i);
                    c_index.Add(3 * this.anchors_index[i] + j);
                }
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
                double dist = verts[i].DistanceToSquared(anchors_position[i]);
                err.Add( strength * dist/(2*edge_avarage_length*edge_avarage_length));
            }
            

            Vec ans = new Vec(err);

            return ans;

        }

        public override bool IsForRigidMode()
        {
            return true;
        }

        protected override System.Drawing.Bitmap Icon
        {
            get
            {
                // You can add image files to your project resources and access them like this:
                //return Resources.IconForThisComponent;
                return Crane.Properties.Resources.anchor;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("74705677-4869-4362-b5d3-61b063eecf74"); }
        }
    }
}
using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class develop : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the develop class.
        /// </summary>
        public develop()
          : base("DevelopMesh", "DevMesh",
              "Description",
              "Crane", "Output")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "M", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;

            if (!DA.GetData(0, ref mesh)) { return; }
            

            var topedge = mesh.TopologyEdges;
            var faces = mesh.Faces;



            //find start face index

            int checkedge = 0;
            int start = 0;
            bool found = false;

            if (!found)
            {
                if(topedge.GetConnectedFaces(checkedge).Length == 1)
                {
                    found = true;
                    start = topedge.GetConnectedFaces(checkedge)[0];
                }
                else
                {
                    checkedge++;
                }
            }

            //make next vertices list empty
            List<Point3f> developedVerts = new List<Point3f>();
            for(int i = 0; i < mesh.Vertices.Count; i++)
            {
                developedVerts.Add(new Point3f());
            }



            

        }

        protected void step(int start,int end,float sAngle,float eAngle,int nextVertIndex)
        {

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
                return null;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("97872d6f-1738-4c83-917f-683e16e5e52b"); }
        }
    }
}
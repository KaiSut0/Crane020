using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

// In order to load the result of this wizard, you will also need to
// add the output bin/ folder of this project to the list of loaded
// folder in Grasshopper.
// You can use the _GrasshopperDeveloperSettings Rhino command for that.

namespace Crane
{
    public class DeconstructCMesh : GH_Component
    {
        /// <summary>
        /// Each implementation of GH_Component must provide a public 
        /// constructor without any arguments.
        /// Category represents the Tab in which the component will appear, 
        /// Subcategory the panel. If you use non-existing tab or panel names, 
        /// new tabs/panels will automatically be created.
        /// </summary>
        public DeconstructCMesh()
          : base("DeconstructCMesh", "DeconstructCMesh",
              "Deconstruct CMesh into Mesh and information",
              "Crane", "Outputs")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "CMesh", GH_ParamAccess.item);
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh", GH_ParamAccess.item);
            pManager.AddLineParameter("Mountain", "M", "Mountain Crease Lines", GH_ParamAccess.list);
            pManager.AddLineParameter("Valley", "V", "Valley Crease Lines", GH_ParamAccess.list);
            pManager.AddLineParameter("Boundary", "B", "Boundary Edges", GH_ParamAccess.list);
            pManager.AddLineParameter("Unassigned", "U", "Unassigned Crease Lines", GH_ParamAccess.list);
            pManager.AddLineParameter("Triangulate", "T", "Triangulate Edges", GH_ParamAccess.list);
            pManager.AddLineParameter("Edges", "E", "Edges", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object can be used to retrieve data from input parameters and 
        /// to store data in output parameters.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            CMesh cmesh = null;

            if (!DA.GetData(0, ref cmesh)) { return; }

            Mesh mesh = ReconstructQuadMesh(cmesh);
            var edges = cmesh.mesh.TopologyEdges;

            List<Line> m = new List<Line>();
            List<Line> v = new List<Line>();
            List<Line> b = new List<Line>();
            List<Line> u = new List<Line>();
            List<Line> t = new List<Line>();
            List<Line> e = new List<Line>();
            

            for(int i = 0; i < edges.Count; i++)
            {
                int info = cmesh.edgeInfo[i];

                if(info == 'M')
                {
                    m.Add(edges.EdgeLine(i));
                }else if(info == 'V')
                {
                    v.Add(edges.EdgeLine(i));
                }else if(info == 'U')
                {
                    u.Add(edges.EdgeLine(i));
                }else if(info == 'B')
                {
                    b.Add(edges.EdgeLine(i));
                }else if(info == 'T')
                {
                    t.Add(edges.EdgeLine(i));
                }

                e.Add(edges.EdgeLine(i));
            }

            DA.SetData(0, mesh);
            DA.SetDataList(1, m);
            DA.SetDataList(2, v);
            DA.SetDataList(3, b);
            DA.SetDataList(4, u);
            DA.SetDataList(5, t);
            DA.SetDataList(6, e);
            
        }

        private Mesh ReconstructQuadMesh(CMesh cm)
        {
            var faces = cm.orig_faces;
            Mesh mesh = cm.mesh.DuplicateMesh();
            mesh.Faces.Destroy();
            mesh.Faces.AddFaces(faces);
            return mesh;
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
                return Crane.Properties.Resources.dec;
            }
        }

        /// <summary>
        /// Each component must have a unique Guid to identify it. 
        /// It is vital this Guid doesn't change otherwise old ghx files 
        /// that use the old ID will partially fail during loading.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("960dd871-c222-45aa-b00a-2fa4c7d36c20"); }
        }
    }
}

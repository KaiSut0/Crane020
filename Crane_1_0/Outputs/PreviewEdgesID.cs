using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino;

namespace Crane
{
    public class PreviewEdgesID : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the PreviewEdgesID class.
        /// </summary>
        public PreviewEdgesID()
          : base("EdgeCentor", "EdgeCentor",
              "EdgeCentor",
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
            pManager.AddPointParameter("EdgesIDLocation", "EdgesIDLocation", "EdgesIDLocation", GH_ParamAccess.list);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            CMesh cmesh = null;

            if (!DA.GetData(0, ref cmesh)) { return; }

            Mesh m = cmesh.mesh;
            List<Point3d> verts = new List<Point3d>(m.Vertices.ToPoint3dArray());

            List<int> edges_id = cmesh.inner_boundary_edges;
            List<Point3d> pts = new List<Point3d>();
            // List<TextDot> id_texts = new List<TextDot>();

            for (int i = 0; i < edges_id.Count; i++)
            {
                IndexPair edge_end_pt_id_pair = m.TopologyEdges.GetTopologyVertices(edges_id[i]);
                Point3d ptI = verts[edge_end_pt_id_pair.I];
                Point3d ptJ = verts[edge_end_pt_id_pair.J];
                Point3d pt = (ptI + ptJ)/2;
                
                // TextDot id_text = new TextDot(i.ToString(), pt);
                // id_texts.Add(id_text);
                pts.Add(pt);
                
            }
            DA.SetDataList(0, pts);
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
                return Crane.Properties.Resources.edge_center;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("b2b8899c-f8b8-4e01-9441-cb5cc1d384cf"); }
        }
    }
}
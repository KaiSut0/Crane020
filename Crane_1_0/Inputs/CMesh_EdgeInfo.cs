using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class CMesh_EdgeInfo : GH_Component
    {
        /// <summary>
        /// Initializes a new instance of the CMesh_EdgeInfo class.
        /// </summary>
        public CMesh_EdgeInfo()
          : base("CMesh_EdgeInfo", "Nickname",
              "Description",
              "Category", "Subcategory")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddMeshParameter("Mesh", "M", "Mesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("Edge Info", "E", "U:unassigned\r\n M:mountain\r\n V:valley", GH_ParamAccess.list);
            
            pManager[1].Optional = true;
        }

        /// <summary>
        /// Registers all the output parameters for this component.
        /// </summary>
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "CMesh", GH_ParamAccess.item);
        }

        /// <summary>
        /// This is the method that actually does the work.
        /// </summary>
        /// <param name="DA">The DA object is used to retrieve from inputs and store in outputs.</param>
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            Mesh mesh = null;
            List<Char> edgeinfo = new List<Char>();

            if (!DA.GetData(0, ref mesh)) { return; }
            DA.GetDataList(1, edgeinfo);

            CMesh cmesh = new CMesh(mesh, edgeinfo);

            DA.SetData(0, cmesh);
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
            get { return new Guid("c31d0133-f785-45db-979a-3a918c02f1e1"); }
        }
    }
}
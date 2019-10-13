using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;
using Rhino.Geometry.Collections;

namespace Crane
{
    public class UpperFoldAngle : GH_Component
    {
        public double upperangle = Math.PI;
        /// <summary>
        /// Initializes a new instance of the UpperFoldAngle class.
        /// </summary>
        public UpperFoldAngle()
          : base("UpperFoldAngle", "UpperFoldAngle",
              "Set upper fold angle.",
              "Crane", "Constraints")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddNumberParameter("UpperAngle", "UA", "Set upper fold angle", GH_ParamAccess.item);
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
            upperangle = Math.PI;
            DA.GetData(0, ref upperangle);
            DA.SetData(0, this);
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
                return Crane.Properties.Resources.upper_fold_angle;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("b5bcfa68-cd67-46e6-90ee-8896783652f3"); }
        }
    }
}
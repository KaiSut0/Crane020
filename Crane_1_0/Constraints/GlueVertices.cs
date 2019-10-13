using System;
using System.Collections.Generic;

using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Crane
{
    public class GlueVertices : Constraint
    {
        List<int> glue_vertices_index = new List<int>();
        public double edge_avarage_length = 0;

        /// <summary>
        /// Initializes a new instance of the GlueVertices class.
        /// </summary>
        public GlueVertices()
          : base("GlueVertices", "GlueVerts",
              "Glue selected vertices",
              "Crane", "Constraints")
        {
        }

        /// <summary>
        /// Registers all the input parameters for this component.
        /// </summary>
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddGenericParameter("CMesh", "CMesh", "Input CMesh", GH_ParamAccess.item);
            pManager.AddIntegerParameter("GlueVerticesID", "GlueVertID", "Input Indices of vertices to Glue", GH_ParamAccess.list);
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
            // 値をセットする毎に public member をリフレッシュ
            glue_vertices_index = new List<int>();
            edge_avarage_length = 0;

            CMesh cmesh = null;

            if (!DA.GetData(0, ref cmesh)) { return; }
            if (!DA.GetDataList(1, glue_vertices_index)) { return; }

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

            for (int i = 0; i < this.glue_vertices_index.Count-1; i++)
            {
                Point3d pti = m.Vertices[this.glue_vertices_index[i]];
                Point3d pti1 = m.Vertices[this.glue_vertices_index[i+1]];

                for (int j = 0; j < 3; j++)
                {
                    var.Add((pti[j] - pti1[j]) / (edge_avarage_length * edge_avarage_length));
                    r_index.Add(i);
                    c_index.Add(3 * this.glue_vertices_index[i] + j);
                    var.Add((pti1[j] - pti[j]) / (edge_avarage_length * edge_avarage_length));
                    r_index.Add(i);
                    c_index.Add(3 * this.glue_vertices_index[i+1] + j);

                }
            }


            CRS Jaco = new CRS(var, r_index, c_index, this.glue_vertices_index.Count-1, verts.Count * 3);

            return Jaco;
        }

        public override Vec Error(CMesh cm)
        {
            List<Point3d> verts = new List<Point3d>(cm.mesh.Vertices.ToPoint3dArray());

            List<double> err = new List<double>();

            for (int i = 0; i < glue_vertices_index.Count-1; i++)
            {
                double dist = verts[glue_vertices_index[i]].DistanceToSquared(verts[glue_vertices_index[i+1]]);
                err.Add(dist / (2 * edge_avarage_length * edge_avarage_length));
            }


            Vec ans = new Vec(err);

            return ans;

        }

        public override bool IsForRigidMode()
        {
            return false;
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
                return Crane.Properties.Resources.glue_vertices;
            }
        }

        /// <summary>
        /// Gets the unique ID for this component. Do not change this ID after release.
        /// </summary>
        public override Guid ComponentGuid
        {
            get { return new Guid("64519271-f9e7-475c-8ef7-12b928e1eecd"); }
        }
    }
}
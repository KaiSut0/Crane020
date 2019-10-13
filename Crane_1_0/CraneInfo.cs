using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace Crane
{
    public class CraneInfo : GH_AssemblyInfo
    {
        public override string Name
        {
            get
            {
                return "Crane";
            }
        }
        public override Bitmap Icon
        {
            get
            {
                //Return a 24x24 pixel bitmap to represent this GHA library.
                return null;
            }
        }
        public override string Description
        {
            get
            {
                //Return a short string describing the purpose of this GHA library.
                return "";
            }
        }
        public override Guid Id
        {
            get
            {
                return new Guid("eb3e078f-3eb3-44e2-8ca2-568b524c8362");
            }
        }

        public override string AuthorName
        {
            get
            {
                //Return a string identifying you or your company.
                return "";
            }
        }
        public override string AuthorContact
        {
            get
            {
                //Return a string representing your preferred contact details.
                return "";
            }
        }
    }
}

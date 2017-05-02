using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Threading.Tasks;

namespace PraxisProjectIuK
{
    public class ChangeTrc
    {
        public string myfile { get; set; }
        public string filename { get; set; }
        public void changeExtensions()
        {
            string txtFileName = @"Output/kinectSkeleton.txt";

            
            File.Move(txtFileName, Path.ChangeExtension(txtFileName , ".trc"));
            File.Move(txtFileName, Path.ChangeExtension(txtFileName, ".mot"));

        }
        
    }
}

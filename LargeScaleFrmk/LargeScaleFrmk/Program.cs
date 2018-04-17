using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace LargeScaleFrmk
{
    class Program
    {
        static void Main(string[] args)
        {
            DateTime startTime = DateTime.Now;
            OriginalModel orgModel = new LargeScaleFrmk.OriginalModel();
            orgModel.Optimize();

            //LagrangianRelaFrmk lrModel = new LagrangianRelaFrmk();
            //lrModel.Optimize();

            //CG cgModel = new CG();
            //cgModel.Optimize();

            //Benders bendersModel = new Benders();
            //bendersModel.Optimize();

            TimeSpan totalRunningTime = DateTime.Now - startTime;

            Console.WriteLine("Total computational time:{0} ms", totalRunningTime.TotalMilliseconds);
            Console.ReadLine();
        }
    }
}

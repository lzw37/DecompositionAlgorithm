using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Gurobi;

namespace LargeScaleFrmk
{
    class ColumnGeneration : ISolver
    {
        const int M = 50000;

        DataStructure Data;

        GRBEnv _env;
        GRBModel _grbModel;


        Dictionary<Node, double> Dual = new Dictionary<Node, double>();
        List<Dictionary<Node, int>> SchemeSet = new List<Dictionary<Node, int>>();

        public void Optimize()
        {
            Data = new DataStructure();
            Data.LoadData();

            Initialize();

            BuildModel_RestrMaster();
            if(SolveMaster())
                ParseSolution();


            Console.WriteLine("Dual:");
            foreach (Node n in Data.NodeSet)
            {
                Console.WriteLine(Dual[n]);
            }

            //BuildModel_Slave();
            //if (SolveSlave())
            //{
            //    foreach (Node n in Data.NodeSet)
            //    {
            //        Console.WriteLine(n.IsServerLocationSelected);
            //    }
            //}
        }

        void Initialize()
        {
            Dictionary<Node, int> initialScheme = new Dictionary<Node, int>();
            SchemeSet.Add(initialScheme);
            foreach (Node n in Data.NodeSet)
            {
                Dual.Add(n, 0);

                if (n.ID == "1" || n.ID == "5")
                {
                    initialScheme.Add(n, 1);
                    n.IsServerLocationSelected = 1;
                }
                else
                {
                    initialScheme.Add(n, 0);
                    n.IsServerLocationSelected = 0;
                }

            }

            initialScheme = new Dictionary<Node, int>();
            SchemeSet.Add(initialScheme);
            foreach (Node n in Data.NodeSet)
            {

                initialScheme.Add(n, 1);
                //n.IsServerLocationSelected = 1;
            }

            initialScheme = new Dictionary<Node, int>();
            SchemeSet.Add(initialScheme);
            foreach (Node n in Data.NodeSet)
            {

                initialScheme.Add(n, 0);
                //n.IsServerLocationSelected = 1;
            }


        }

        void BuildModel_RestrMaster()
        {
            _env = new GRBEnv("SolutionLog.log");
            _env.Set(GRB.DoubleParam.MIPGap, 0.0);
            _env.Set(GRB.DoubleParam.TimeLimit, 500);
            _grbModel = new GRBModel(_env);

            //决策变量
            foreach (Node n in Data.NodeSet)
            {
                n.Result_GenerateFlow = _grbModel.AddVar(0.0, M, 0.0, GRB.CONTINUOUS, "g_" + n.ID);
            }

            foreach (Dictionary<Node,int> scheme in SchemeSet)
            {
                _grbModel.AddVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "k_" + SchemeSet.IndexOf(scheme));
            }

            foreach (Arc a in Data.ArcSet)
            {
                a.Result_FlowF = _grbModel.AddVar(0.0, M, 0.0, GRB.CONTINUOUS, "fF_" + a.FromNode.ID + "_" + a.ToNode.ID);
                a.Result_FlowR = _grbModel.AddVar(0.0, M, 0.0, GRB.CONTINUOUS, "fR_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
            _grbModel.Update();

            //目标函数
            GRBLinExpr expr1 = 0;
            foreach (Dictionary<Node, int> scheme in SchemeSet)
            {
                foreach (Node n in Data.NodeSet)
                    expr1 += scheme[n] * Data.ServerInstalationFee * _grbModel.GetVarByName("k_" + SchemeSet.IndexOf(scheme));
            }
            GRBLinExpr expr2 = 0;
            foreach (Arc a in Data.ArcSet)
                expr2 += (a.Result_FlowF + a.Result_FlowR) * Data.FlowFeePerUnit;

            _grbModel.SetObjective(expr1 + expr2, GRB.MINIMIZE);


            //约束条件

            foreach (Node n in Data.NodeSet)
            {
                GRBLinExpr expr = 0;
                foreach (Dictionary<Node, int> scheme in SchemeSet)
                {
                    expr += scheme[n] * _grbModel.GetVarByName("k_" + SchemeSet.IndexOf(scheme)) * M;
                }
                _grbModel.AddConstr(expr - n.Result_GenerateFlow >= 0, "ct1_" + n.ID);
            }

            foreach (Node n in Data.NodeSet)
            {
                GRBLinExpr sum1 = 0;
                GRBLinExpr sum2 = 0;
                GRBLinExpr sum3 = 0;
                GRBLinExpr sum4 = 0;
                foreach (Arc a in n.ArcSet)
                {
                    if (a.ToNode == n)//进
                    {
                        sum1 += a.Result_FlowF;
                        sum3 += a.Result_FlowR;
                    }
                    else//出
                    {
                        sum2 += a.Result_FlowR;
                        sum4 += a.Result_FlowF;
                    }
                }
                _grbModel.AddConstr(n.Result_GenerateFlow + sum1 + sum2 == n.Demand + sum3 + sum4, "ct2_" + n.ID);
            }
            foreach (Arc a in Data.ArcSet)
            {
                _grbModel.AddConstr(a.Result_FlowF + a.Result_FlowR <= a.Capacity, "ct3_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }

            GRBLinExpr exprCtr = 0;
            foreach (Dictionary<Node, int> scheme in SchemeSet)
            {
                exprCtr += _grbModel.GetVarByName("k_" + SchemeSet.IndexOf(scheme));
            }
            _grbModel.AddConstr(exprCtr == 1, "ct4");
        }
        bool SolveMaster()
        {
            _grbModel.Optimize();
            int status = _grbModel.Get(GRB.IntAttr.Status);
            int solution = _grbModel.Get(GRB.IntAttr.SolCount);
            if (status == GRB.Status.OPTIMAL || (status == GRB.Status.TIME_LIMIT && solution > 0))
            {
                foreach (Node n in Data.NodeSet)
                {
                    GRBConstr constr = _grbModel.GetConstrByName("ct1_" + n.ID);
                    Dual[n] = constr.Get(GRB.DoubleAttr.Pi);
                    n.ParseSolution(2);
                    n.ParseSolution(1);
                }
                foreach (Arc a in Data.ArcSet)
                {
                    a.ParseSolution();
                }

                double k0 = _grbModel.GetVarByName("k_0").Get(GRB.DoubleAttr.X);
                double k1 = _grbModel.GetVarByName("k_1").Get(GRB.DoubleAttr.X);

                return true;
            }
            else
            {
                return false;
            }
        }
        void BuildModel_Slave()
        {
            //决策变量
            foreach (Node n in Data.NodeSet)
            {
                n.Result_IsServerLoacationSelected = _grbModel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + n.ID);
            }
            _grbModel.Update();

            //目标函数
            GRBLinExpr expr1 = 0;
            foreach (Node n in Data.NodeSet)
                expr1 += n.Result_IsServerLoacationSelected * Data.ServerInstalationFee;
            GRBLinExpr expr2 = 0;
            foreach (Node n in Data.NodeSet)
                expr2 += Dual[n] * n.Result_IsServerLoacationSelected;

            _grbModel.SetObjective(expr2 - expr1, GRB.MAXIMIZE);

            
        }

        bool SolveSlave()
        {
            _grbModel.Optimize();
            int status = _grbModel.Get(GRB.IntAttr.Status);
            int solution = _grbModel.Get(GRB.IntAttr.SolCount);
            if (status == GRB.Status.OPTIMAL || (status == GRB.Status.TIME_LIMIT && solution > 0))
            {
                double objValue = _grbModel.Get(GRB.DoubleAttr.ObjVal);
                if (objValue <= 0)
                {
                }
                foreach (Node n in Data.NodeSet)
                {
                    n.ParseSolution(1);
                }
                Dictionary<Node, int> newScheme = new Dictionary<Node, int>();
                foreach (Node n in Data.NodeSet)
                {
                    newScheme.Add(n, Convert.ToInt32(n.IsServerLocationSelected));
                    SchemeSet.Add(newScheme);
                }
                return true;
            }
            else
                return false;
        }

        void ParseSolution()
        {
            Console.WriteLine("NODE ID\tSELECT\tGENERATE FLOW");
            foreach (Node n in Data.NodeSet)
            {
                n.ParseSolution(1);
                n.ParseSolution(2);

                Console.WriteLine("{0}\t{1}\t{2}", n.ID, n.IsServerLocationSelected, n.GenerateFlow);
            }
            Console.WriteLine();
            Console.WriteLine("FROM ID\tTO ID\tFLOW F\tFLOW R");
            foreach (Arc a in Data.ArcSet)
            {
                a.ParseSolution();
                Console.WriteLine("{0}\t{1}\t{2}\t{3}", a.FromNode.ID, a.ToNode.ID, a.FlowF, a.FlowR);
            }
        }
    }
}

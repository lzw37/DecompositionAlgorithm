using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Gurobi;
using System.IO;

namespace LargeScaleFrmk
{
    class LagrangianRelaFrmk
    {
        DataStructure Data;

        GRBEnv _env;
        GRBModel _grbModel;


        Dictionary<Node, double> multiplier_p = new Dictionary<Node, double>();
        public int IterationTimes = 500;

        public void Optimize()
        {

            Data = new DataStructure();
            Data.LoadData();

            //生成乘子
            foreach (Node n in Data.NodeSet)
            {
                multiplier_p.Add(n, 0);
            }
           
            for (int i = 1; i <= IterationTimes; i++)
            {
                double LB = 0;
                BuildModel_SubProblem1();
                if (Solve())
                {
                    ParseSolution_SubProblem1();
                    LB += _grbModel.Get(GRB.DoubleAttr.ObjVal);
                }
                _grbModel.Dispose();
                _env.Dispose();

              
                BuildModel_SubProblem2();
                if (Solve())
                {
                    ParseSolution_SubProblem2();
                    LB += _grbModel.Get(GRB.DoubleAttr.ObjVal);
                }

                _grbModel.Dispose();
                _env.Dispose();
                UpdateMultiplier(i);


                BuildFeasibleSolutionModel();
                if (Solve())
                {
                    ParseSolution_SubProblem2();
                    StreamWriter sw = File.AppendText("solution.csv");
                    sw.Write("{0},{1},{2}", i, _grbModel.Get(GRB.DoubleAttr.ObjVal), LB);
                    sw.WriteLine();
                    sw.Close();
                }
                else
                {
                    StreamWriter sw = File.AppendText("solution.csv");
                    sw.Write("{0},{1},{2}", i, "infeasible", LB);
                    sw.WriteLine();
                    sw.Close();
                }
                _grbModel.Dispose();
                _env.Dispose();

            }


            ParseSolution();
            
        }

        void BuildModel_SubProblem1()
        {
            _env = new GRBEnv("SolutionLog.log");
            _env.Set(GRB.DoubleParam.MIPGap, 0.0);
            _env.Set(GRB.DoubleParam.TimeLimit, 500);
            _grbModel = new GRBModel(_env);
            //决策变量
            foreach (Node n in Data.NodeSet)
            {
                n.Result_IsServerLoacationSelected = _grbModel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + n.ID);
            }
            _grbModel.Update();

            //目标函数
            GRBLinExpr expr1 = 0;
            foreach (Node n in Data.NodeSet)
                expr1 += n.Result_IsServerLoacationSelected * (Data.ServerInstalationFee - multiplier_p[n] * Data.M);
            _grbModel.SetObjective(expr1, GRB.MINIMIZE);

        }

        void BuildModel_SubProblem2()
        {
            _env = new GRBEnv("SolutionLog.log");
            _env.Set(GRB.DoubleParam.MIPGap, 0.0);
            _env.Set(GRB.DoubleParam.TimeLimit, 500);
            _grbModel = new GRBModel(_env);

            //决策变量
            foreach (Node n in Data.NodeSet)
            {
                n.Result_GenerateFlow = _grbModel.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "g_" + n.ID);
            }
            foreach (Arc a in Data.ArcSet)
            {
                a.Result_FlowF = _grbModel.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "fF_" + a.FromNode.ID + "_" + a.ToNode.ID);
                a.Result_FlowR = _grbModel.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "fR_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
            _grbModel.Update();

            //目标函数
            GRBLinExpr expr1 = 0;
            foreach (Node n in Data.NodeSet)
                expr1 += n.Result_GenerateFlow * multiplier_p[n];
            GRBLinExpr expr2 = 0;
            foreach (Arc a in Data.ArcSet)
                expr2 += (a.Result_FlowF + a.Result_FlowR) * Data.FlowFeePerUnit;

            _grbModel.SetObjective(expr1 + expr2, GRB.MINIMIZE);


            //约束条件
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
        }



        void UpdateMultiplier(double currentIteration)
        {
            foreach(Node n in Data.NodeSet)
            {
                double newMultiplier = multiplier_p[n] + (n.GenerateFlow - n.IsServerLocationSelected * Data.M) * (0.004 / (1 + currentIteration));
                if (newMultiplier < 0)
                    multiplier_p[n] = 0;
                else
                    multiplier_p[n] = newMultiplier;
            }
        }

        void BuildFeasibleSolutionModel()
        {
            _env = new GRBEnv("SolutionLog.log");
            _env.Set(GRB.DoubleParam.MIPGap, 0.0);
            _env.Set(GRB.DoubleParam.TimeLimit, 500);
            _grbModel = new GRBModel(_env);

            //决策变量
            foreach (Node n in Data.NodeSet)
            {
                n.Result_GenerateFlow = _grbModel.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "g_" + n.ID);
            }
            foreach (Arc a in Data.ArcSet)
            {
                a.Result_FlowF = _grbModel.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "fF_" + a.FromNode.ID + "_" + a.ToNode.ID);
                a.Result_FlowR = _grbModel.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "fR_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
            _grbModel.Update();

            //目标函数
            GRBLinExpr expr1 = 0;
            foreach (Node n in Data.NodeSet)
                expr1 += n.IsServerLocationSelected * Data.ServerInstalationFee;
            GRBLinExpr expr2 = 0;
            foreach (Arc a in Data.ArcSet)
                expr2 += (a.Result_FlowF + a.Result_FlowR) * Data.FlowFeePerUnit;

            _grbModel.SetObjective(expr1 + expr2, GRB.MINIMIZE);


            //约束条件
            foreach (Node n in Data.NodeSet)
                _grbModel.AddConstr(n.Result_GenerateFlow <= n.IsServerLocationSelected * Data.M, "ct1_" + n.ID);
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
        }


        bool Solve()
        {
            _grbModel.Optimize();
            int status = _grbModel.Get(GRB.IntAttr.Status);
            int solution = _grbModel.Get(GRB.IntAttr.SolCount);
            if (status == GRB.Status.OPTIMAL || (status == GRB.Status.TIME_LIMIT && solution > 0))
                return true;
            else
                return false;
        }

        void ParseSolution_SubProblem1()
        {
            foreach (Node n in Data.NodeSet)
                n.ParseSolution(1);
        }

        void ParseSolution_SubProblem2()
        {
            foreach (Node n in Data.NodeSet)
                n.ParseSolution(2);
            foreach (Arc a in Data.ArcSet)
                a.ParseSolution();
        }


        void ParseSolution()
        {
            Console.WriteLine("NODE ID\tSELECT\tGENERATE FLOW\tMULTIPLIER");
            foreach (Node n in Data.NodeSet)
            {
                Console.WriteLine("{0}\t{1}\t{2}\t{3}", n.ID, n.IsServerLocationSelected, n.GenerateFlow, multiplier_p[n]);
            }
            Console.WriteLine();
            Console.WriteLine("FROM ID\tTO ID\tFLOW F\tFLOW R");
            foreach (Arc a in Data.ArcSet)
            {
                Console.WriteLine("{0}\t{1}\t{2}\t{3}", a.FromNode.ID, a.ToNode.ID, a.FlowF, a.FlowR);
            }
        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Gurobi;

namespace LargeScaleFrmk
{

    //数学模型Latex代码
    //Minimize \sum_{i \in N}(x_i \times C_i)+\sum_{(i,j) \in A}[c_{i, j} \times(f_{ (i, j)}^F+f_{(i,j)}^R)]

    //s.t.
    /*
    g_i \leq x_i \times M, \forall i \in N
    g_j+\sum_{i:(i, j) \in A}f_{(i,j)}^F + \sum_{k:(j,k) \in A}f_{(j,k)}^R =
    D_j+\sum_{i:(i,j) \in A}f_{(i,j)}^R+ \sum_{k:(j,k)\in A}f_{(j,k)}^F , \forall j \in N
    f_{(i, j)}^F+f_{(i,j)}^R \leq V_ {(i,j)} , \forall(i, j) \in A
    */


    class OriginalModel
    {
        DataStructure Data;

        GRBEnv _env;
        GRBModel _grbModel;

        void BuildGRBModel()
        {
            _env = new GRBEnv("SolutionLog.log");
            _env.Set(GRB.DoubleParam.MIPGap, 0.0);
            _env.Set(GRB.DoubleParam.TimeLimit, 500);
            _env.Set(GRB.DoubleParam.Heuristics, 0.5);
            _grbModel = new GRBModel(_env);

            //决策变量
            foreach (Node n in Data.NodeSet)
            {
                n.Result_GenerateFlow = _grbModel.AddVar(0.0, Data.M, 0.0, GRB.CONTINUOUS, "g_" + n.ID);
                n.Result_IsServerLoacationSelected = _grbModel.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "x_" + n.ID);
            }
            foreach (Arc a in Data.ArcSet)
            {
                a.Result_FlowF = _grbModel.AddVar(0.0, Data.M, 0.0, GRB.CONTINUOUS, "fF_" + a.FromNode.ID + "_" + a.ToNode.ID);
                a.Result_FlowR = _grbModel.AddVar(0.0, Data.M, 0.0, GRB.CONTINUOUS, "fR_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
            _grbModel.Update();

            //目标函数
            GRBLinExpr expr1 = 0;
            foreach (Node n in Data.NodeSet)
                expr1 += n.Result_IsServerLoacationSelected * Data.ServerInstalationFee;
            GRBLinExpr expr2 = 0;
            foreach (Arc a in Data.ArcSet)
                expr2 += (a.Result_FlowF + a.Result_FlowR) * Data.FlowFeePerUnit;

            _grbModel.SetObjective(expr1 + expr2, GRB.MINIMIZE);


            //约束条件
            foreach (Node n in Data.NodeSet)
                _grbModel.AddConstr(n.Result_GenerateFlow <= n.Result_IsServerLoacationSelected * Data.M, "ct1_" + n.ID);
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
            _grbModel.Write("OrgModel.lp");
            _grbModel.Optimize();

            int status = _grbModel.Get(GRB.IntAttr.Status);
            int solution = _grbModel.Get(GRB.IntAttr.SolCount);
            if (status == GRB.Status.OPTIMAL || (status == GRB.Status.TIME_LIMIT && solution > 0))
                return true;
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
        public void Optimize()
        {
            Data = new DataStructure();
            Data.LoadData();
            BuildGRBModel();
            if (Solve())
                ParseSolution();
            _grbModel.Dispose();
            _env.Dispose();
        }
    }
}

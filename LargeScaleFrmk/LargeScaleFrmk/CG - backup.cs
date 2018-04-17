using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Gurobi;

namespace LargeScaleFrmk
{
    class CG : ISolver
    {
        class Scheme
        {
            public int ID;

            public Dictionary<Node, int> Value = new Dictionary<Node, int>();

            public double GetTotalInstallationFee(double installFee)
            {
                double total = 0;
                foreach (Node n in Value.Keys)
                {
                    total += Value[n] * installFee;
                }
                return total;
            }
        }

        class ExtremePoint
        {
            public int ID;
            public Dictionary<Node, double> Value = new Dictionary<Node, double>();
            public double TotalCost;
        }

        class BranchNode
        {
            public BranchNode ParentNode;
            public BranchNode LeftChild;
            public BranchNode RightChild;

            public Dictionary<Node, double> ConstrPool;
            public double LB;
        }

        List<Scheme> SchemeColumnPool = new List<Scheme>();
        List<ExtremePoint> EPColumnPool = new List<ExtremePoint>();

        Dictionary<Node, double> DualSolution = new Dictionary<Node, double>();
        double SchemeDualValue = 0;
        double FADualValue = 0;

        DataStructure Data = new DataStructure();



        GRBEnv _envMaster;
        GRBModel _grbModelMaster;


        public void Optimize()
        {
            Data.LoadData();

            GenerateInitialFeasibleSolution();

            while (true)
            {

                _envMaster = new GRBEnv("CG_Model.log");
                _grbModelMaster = new GRBModel(_envMaster);


                BuildVar();
                GenerateConst();

                _grbModelMaster.Write("CG_Model.lp");
                _grbModelMaster.Optimize();

                GetDual();

                OutputSolution();

                OptimizeSchemeSub();
                OptimizeFASub();

                bool IsActive = AddColumn();

                _grbModelMaster.Dispose();
                _envMaster.Dispose();
                _grbModel_SchemeGenerateSub.Dispose();
                _env_SchemeGenerateSub.Dispose();
                _grbModel_FlowAssignmentSub.Dispose();
                _env_FlowAssignmentSub.Dispose();

                if (!IsActive)
                {
                    break;
                }
            }
        }

        private void OutputSolution()
        {
            GRBVar[] varArray = _grbModelMaster.GetVars();
            GRBConstr[] constArray = _grbModelMaster.GetConstrs();
            foreach (GRBVar v in varArray)
            {
                Console.WriteLine("{0}:{1}", v.VarName, v.X);
            }
            foreach (GRBConstr ct in constArray)
            {
                Console.WriteLine("{0}:{1}", ct.ConstrName, ct.Pi);
            }
        }

        private void GetDual()
        {
            SchemeDualValue = _grbModelMaster.GetConstrByName("ct1").Pi;
            FADualValue = _grbModelMaster.GetConstrByName("ct2").Pi;

            DualSolution.Clear();
            foreach (Node n in Data.NodeSet)
            {
                GRBConstr constr = _grbModelMaster.GetConstrByName("ct3_" + n.ID);
                double dualValue = constr.Pi;
                DualSolution.Add(n, dualValue);
            }
        }

        #region 分支定界
        private void BranchAndBound()
        {

        }

        private void BuildBranchingTree(BranchNode branchNode)
        {
            Dictionary<Node, double> nodeTestingValue = new Dictionary<Node, double>();
            foreach (Node n in Data.NodeSet)
            {
                double nodeValue = 0;
                foreach (Scheme s in SchemeColumnPool)
                {
                    double var = _grbModelMaster.GetVarByName("lambda_" + s.ID).X;
                    nodeValue += s.Value[n] * var;
                }
                nodeTestingValue.Add(n, nodeValue);
            }
            Node currentNode = nodeTestingValue.Max().Key;


            BranchNode leftBranchNode = new BranchNode();/*优势分支*/
            branchNode.LeftChild = leftBranchNode;
            leftBranchNode.ParentNode = branchNode;

            leftBranchNode.ConstrPool = new Dictionary<Node, double>(branchNode.ConstrPool);/*generate constraint pool*/
            if (nodeTestingValue[currentNode]>0.5)
            {
                leftBranchNode.ConstrPool.Add(currentNode, 1);
            }
            else
            {
                leftBranchNode.ConstrPool.Add(currentNode, 0);
            }                                             
            Branching(leftBranchNode);

            BranchNode rightBranchNode = new BranchNode();/*劣势分支*/
            rightBranchNode.ConstrPool = new Dictionary<Node, double>(branchNode.ConstrPool);/*generate constraint pool*/
            if (nodeTestingValue[currentNode] > 0.5)
            {
                rightBranchNode.ConstrPool.Add(currentNode, 1);
            }
            else
            {
                rightBranchNode.ConstrPool.Add(currentNode, 0);
            }
            Branching(rightBranchNode);
        }
        private void Branching(BranchNode branchingNode)
        {
            /*CG, Update LB of this branch, cut decision*/


            /*feasibility test, Update global UB, stop criteria*/

            BuildBranchingTree(branchingNode);
        }
        #endregion




        #region 限制主问题

        private void BuildVar()
        {
            foreach (Scheme s in SchemeColumnPool)
            {
                _grbModelMaster.AddVar(0.0, 1.0, s.GetTotalInstallationFee(Data.ServerInstalationFee), GRB.CONTINUOUS, "lambda_" + s.ID);
            }
            foreach (ExtremePoint e in EPColumnPool)
            {
                _grbModelMaster.AddVar(0.0, 1.0, e.TotalCost, GRB.CONTINUOUS, "mu_" + e.ID);
            }
            _grbModelMaster.Update();
        }
        public void GenerateInitialFeasibleSolution()
        {

            //foreach (Node n in Data.NodeSet)
            //{
            //    Scheme s = new LargeScaleFrmk.CG.Scheme();
            //    foreach (Node n2 in Data.NodeSet)
            //    {
            //        if (n == n2)
            //        {
            //            s.Value.Add(n2, 1);
            //            continue;
            //        }
            //        s.Value.Add(n2, 0);
            //    }
            //    s.ID = SchemeColumnPool.Count;
            //    SchemeColumnPool.Add(s);
            //}

            Scheme s = new LargeScaleFrmk.CG.Scheme();
            foreach (Node n2 in Data.NodeSet)
            {
                s.Value.Add(n2, 1);
            }
            s.ID = SchemeColumnPool.Count;
            SchemeColumnPool.Add(s);


            //s = new LargeScaleFrmk.CG.Scheme();
            //foreach (Node n2 in Data.NodeSet)
            //{
            //    if (n2.ID == "1")
            //    {
            //        s.Value.Add(n2, 1);
            //        continue;
            //    }
            //    s.Value.Add(n2, 0);
            //}
            //s.ID = SchemeColumnPool.Count;
            //SchemeColumnPool.Add(s);






            ExtremePoint e = new ExtremePoint();
            foreach (Node n in Data.NodeSet)
            {
                e.Value.Add(n, n.Demand);
            }
            e.ID = 0;
            e.TotalCost = 0;
            EPColumnPool.Add(e);
        }

        private void GenerateConst()
        {
            GRBLinExpr expr1 = 0;
            foreach (Scheme s in SchemeColumnPool)
            {
                expr1 += _grbModelMaster.GetVarByName("lambda_" + s.ID);
            }
            _grbModelMaster.AddConstr(expr1 == 1, "ct1");

            GRBLinExpr expr2 = 0;
            foreach (ExtremePoint e in EPColumnPool)
            {
                expr2 += _grbModelMaster.GetVarByName("mu_" + e.ID);
            }
            _grbModelMaster.AddConstr(expr2 == 1, "ct2");

            foreach (Node n in Data.NodeSet)
            {
                GRBLinExpr expr = 0;
                foreach (Scheme s in SchemeColumnPool)
                {
                    GRBVar lambda = _grbModelMaster.GetVarByName("lambda_" + s.ID);
                    expr += Data.ServerCapacity * s.Value[n] * lambda;
                }
                foreach (ExtremePoint e in EPColumnPool)
                {
                    GRBVar mu = _grbModelMaster.GetVarByName("mu_" + e.ID);
                    expr += -e.Value[n] * mu;
                }
                _grbModelMaster.AddConstr(expr >= 0, "ct3_" + n.ID);
            }
        }


        #endregion


        #region 定价子问题
        GRBEnv _env_SchemeGenerateSub;
        GRBModel _grbModel_SchemeGenerateSub;

        private void OptimizeSchemeSub()
        {
            _env_SchemeGenerateSub = new GRBEnv("SchemeSub.log");
            _grbModel_SchemeGenerateSub = new GRBModel(_env_SchemeGenerateSub);

            BuildVar_SG();
            BuildObj_SG();
            BuildConst_SG();

            _grbModel_SchemeGenerateSub.Write("SchemeSub.lp");

            _grbModel_SchemeGenerateSub.Optimize();

            OutputSolution_SchemeSub();

            //_grbModel_SchemeGenerateSub.Dispose();
            //_env_SchemeGenerateSub.Dispose();
        }

        private void OutputSolution_SchemeSub()
        {
            Console.WriteLine("SchemeSub----");
            GRBVar[] varArray = _grbModel_SchemeGenerateSub.GetVars();
            foreach (GRBVar v in varArray)
            {
                Console.WriteLine("{0}:{1}", v.VarName, v.X);
            }
        }

        private bool AddColumn()
        {
            if (_grbModel_SchemeGenerateSub.ObjVal > _grbModel_FlowAssignmentSub.ObjVal)
            {
                if (_grbModel_SchemeGenerateSub.ObjVal > 0)
                {
                    Scheme s = new Scheme();
                    foreach (Node n in Data.NodeSet)
                    {
                        double Y = _grbModel_SchemeGenerateSub.GetVarByName("y_" + n.ID).X;

                        s.Value.Add(n, Convert.ToInt32(Y));
                    }
                    s.ID = SchemeColumnPool.Count;
                    SchemeColumnPool.Add(s);
                    return true;
                }
                else
                    return false;
            }
            else
            {
                if (_grbModel_FlowAssignmentSub.ObjVal > 0)
                {
                    foreach (Node n in Data.NodeSet)
                    {
                        ExtremePoint e = new ExtremePoint();
                        double G = _grbModel_FlowAssignmentSub.GetVarByName("g_" + n.ID).X;
                        e.Value.Add(n, G);
                    }
                    return true;
                }
                else
                    return false;
            }
        }

        private void BuildVar_SG()
        {
            foreach (Node n in Data.NodeSet)
            {
                GRBVar y = _grbModel_SchemeGenerateSub.AddVar(0.0, 1.0, 0.0, GRB.BINARY, "y_" + n.ID);
            }
            _grbModel_SchemeGenerateSub.Update();
        }

        private void BuildObj_SG()
        {
            GRBLinExpr expr = 0;
            expr += SchemeDualValue * 1 + FADualValue * 0;
            foreach (Node n in Data.NodeSet)
            {
                expr += DualSolution[n] * Data.ServerCapacity * _grbModel_SchemeGenerateSub.GetVarByName("y_" + n.ID);
            }
            foreach (Node n in Data.NodeSet)
            {
                expr += -Data.ServerInstalationFee * _grbModel_SchemeGenerateSub.GetVarByName("y_" + n.ID);
            }
            _grbModel_SchemeGenerateSub.SetObjective(expr, GRB.MAXIMIZE);
        }
        private void BuildConst_SG()
        {
            GRBLinExpr expr = 0;
            foreach (Node n in Data.NodeSet)
            {
                expr += _grbModel_SchemeGenerateSub.GetVarByName("y_" + n.ID);
            }
            _grbModel_SchemeGenerateSub.AddConstr(expr >= 1, "ct_1");
        }


        GRBEnv _env_FlowAssignmentSub;
        GRBModel _grbModel_FlowAssignmentSub;

        private void OptimizeFASub()
        {
            _env_FlowAssignmentSub = new GRBEnv("FASub.log");
            _grbModel_FlowAssignmentSub = new GRBModel(_env_FlowAssignmentSub);


            BuildVar_FA();
            BuildObj_FA();
            BuildConst_FA();

            _grbModel_FlowAssignmentSub.Write("FASub.lp");

            _grbModel_FlowAssignmentSub.Optimize();
            OutputValue();

        }
        private void OutputValue()
        {
            Console.WriteLine("FASub----");
            GRBVar[] varArray = _grbModel_FlowAssignmentSub.GetVars();
            foreach (GRBVar v in varArray)
            {
                Console.WriteLine("{0}:{1}", v.VarName, v.X);
            }
        }
        private void BuildVar_FA()
        {
            foreach (Arc a in Data.ArcSet)
            {
                GRBVar x = _grbModel_FlowAssignmentSub.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "x_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
            foreach (Node n in Data.NodeSet)
            {
                GRBVar g = _grbModel_FlowAssignmentSub.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "g_" + n.ID);
                //GRBVar d = _grbModel_FlowAssignmentSub.AddVar(0.0, 1.0, 0.0, GRB.CONTINUOUS, "d_" + n.ID);
            }
            _grbModel_FlowAssignmentSub.Update();
        }
        private void BuildObj_FA()
        {
            GRBLinExpr expr1 = 0;
            expr1 += SchemeDualValue * 0 + FADualValue * 1;
            foreach (Node n in Data.NodeSet)
            {
                expr1 += DualSolution[n] * (-_grbModel_FlowAssignmentSub.GetVarByName("g_" + n.ID));
            }

            foreach (Arc a in Data.ArcSet)
            {
                expr1 += -Data.FlowFeePerUnit * _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
            _grbModel_FlowAssignmentSub.SetObjective(expr1, GRB.MAXIMIZE);
        }
        private void BuildConst_FA()
        {
            BuildFlowBalanceConst();
            BuildCapacityConst();
        }
        private void BuildFlowBalanceConst()
        {
            foreach (Node n in Data.NodeSet)
            {
                GRBLinExpr exprLeft = 0;
                foreach (Arc a in Data.ArcSet)
                {
                    if (a.FromNode == n)
                    {
                        GRBVar x = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
                        exprLeft += x;
                    }
                }
                exprLeft += n.Demand;
                GRBLinExpr exprRight = 0;
                foreach (Arc a in Data.ArcSet)
                {
                    if (a.ToNode == n)
                    {
                        GRBVar x = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
                        exprRight += x;
                    }
                }
                exprRight += _grbModel_FlowAssignmentSub.GetVarByName("g_" + n.ID);

                _grbModel_FlowAssignmentSub.AddConstr(exprLeft == exprRight, "ct_FlBal_" + n.ID);
            }
        }

        private void BuildCapacityConst()
        {
            foreach (Arc a in Data.ArcSet)
            {
                GRBVar x = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
                _grbModel_FlowAssignmentSub.AddConstr(x <= a.Capacity, "ct_Cap_" + a.FromNode.ID + "_" + a.ToNode.ID);
            }
        }
        #endregion
    }
}

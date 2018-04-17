using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Gurobi;
using System.IO;

namespace LargeScaleFrmk
{
    class CG
    {

        DataStructure Data;
        double GlobalUB = 2000000;
        Dictionary<Node, double> OptimalConstrPool;

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

        class ExtremePoint : IDisposable
        {
            public int ID;
            public Dictionary<Node, double> Value = new Dictionary<Node, double>();
            public double TotalCost;
            public void Dispose()
            {
                Value = null;
            }
        }

        class BranchNode
        {
            public BranchNode ParentNode;
            public BranchNode LeftChild;
            public BranchNode RightChild;

            public Dictionary<Node, double> ConstrPool;
            public double LB;
            public CGFramework ColumnGenerationFrmk;
        }


        class CGFramework
        {
            public List<Scheme> SchemeColumnPool = new List<Scheme>();
            public List<ExtremePoint> EPColumnPool = new List<ExtremePoint>();

            Dictionary<Node, double> DualSolution = new Dictionary<Node, double>();
            double SchemeDualValue = 0;
            double FADualValue = 0;
            GRBEnv _envMaster;
            GRBModel _grbModelMaster;

            public DataStructure Data = new DataStructure();

            public BranchNode CurrentBranchNode;

            public double Optimize()
            {
                ///*输出当前节点标识*/
                //StreamWriter sw = File.AppendText("NodeLabel.txt");
                //sw.WriteLine("-----***-----");
                //foreach (Node n in CurrentBranchNode.ConstrPool.Keys)
                //{
                //    sw.WriteLine("{0},{1}", n.ID, CurrentBranchNode.ConstrPool[n]);
                //}
                //sw.WriteLine("---");
                //sw.Close();


                /*查看某一分支*/
                //string idSequence = "";
                //string valueSequence = "";
                //foreach (Node n in CurrentBranchNode.ConstrPool.Keys)
                //{
                //    idSequence += n.ID;
                //    valueSequence += CurrentBranchNode.ConstrPool[n];
                //}
                //if(idSequence=="4123" && valueSequence=="1100")
                //{

                //}

                if (!GenerateInitialFeasibleSolution())
                    return -1;
                double objVal = -1;

                while (true)
                {
                    _envMaster = new GRBEnv(/*"CG_Model.log"*/);
                    _grbModelMaster = new GRBModel(_envMaster);


                    BuildVar();
                    GenerateConst();

                    _grbModelMaster.Write("CG_Model.lp");
                    _grbModelMaster.Optimize();

                    if(_grbModelMaster.Status==GRB.Status.INFEASIBLE)
                    {
                        _grbModelMaster.Dispose();
                        _envMaster.Dispose();
                        return -1;
                    }

                    GetDual();

                    OutputSolution();

                    OptimizeSchemeSub();
                    OptimizeFASub();

                    bool IsActive = AddColumn();
                    objVal = _grbModelMaster.ObjVal;

                    

                    if (!IsActive)
                    {
                        break;
                    }
                }

                /*输出当前节点标识*/
                StreamWriter sw = File.AppendText("NodeLabel.txt");
                sw.WriteLine("-----***-----");
                foreach (Node n in CurrentBranchNode.ConstrPool.Keys)
                {
                    sw.WriteLine("{0},{1},{2}", n.ID, CurrentBranchNode.ConstrPool[n],objVal);
                }
                sw.WriteLine("---");
                sw.Close();
                /*************/

                return objVal;
            }
            public void DisposeModels()
            {
                _grbModelMaster.Dispose();
                _envMaster.Dispose();
                _grbModel_SchemeGenerateSub.Dispose();
                _env_SchemeGenerateSub.Dispose();
                _grbModel_FlowAssignmentSub.Dispose();
                _env_FlowAssignmentSub.Dispose();
                DualSolution.Clear();
            }

            public double GetLambda(Scheme s)
            {
                return _grbModelMaster.GetVarByName("lambda_" + s.ID).X;
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
            public bool GenerateInitialFeasibleSolution()
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
                    if (CurrentBranchNode.ConstrPool.ContainsKey(n2))
                    {
                        s.Value.Add(n2, Convert.ToInt32(CurrentBranchNode.ConstrPool[n2]));
                    }
                    else
                    {
                        s.Value.Add(n2, 1);
                    }
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


                _env_FlowAssignmentSub = new GRBEnv();
                _grbModel_FlowAssignmentSub = new GRBModel(_env_FlowAssignmentSub);

                BuildVar_FA();
                //BuildObj_FA();
                BuildConst_FA();

                _grbModel_FlowAssignmentSub.Write("FASub.lp");

                _grbModel_FlowAssignmentSub.Optimize();

                if(_grbModel_FlowAssignmentSub.Status==GRB.Status.INFEASIBLE)
                {
                    _grbModel_FlowAssignmentSub.Dispose();
                    _env_FlowAssignmentSub.Dispose();
                    return false;
                }


                ExtremePoint e = new ExtremePoint();
                foreach (Node n in Data.NodeSet)
                {
                    double value = _grbModel_FlowAssignmentSub.GetVarByName("g_" + n.ID).X;
                    e.Value.Add(n, value);
                }
                foreach(Arc a in Data.ArcSet)
                {
                    double flow = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID).X +
                        _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.ToNode.ID + "_" + a.FromNode.ID).X;
                    e.TotalCost += flow * Data.FlowFeePerUnit;
                }
                e.ID = 0;
                
                EPColumnPool.Add(e);

                _grbModel_FlowAssignmentSub.Dispose();
                _env_FlowAssignmentSub.Dispose();

                return true;
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
                _env_SchemeGenerateSub = new GRBEnv(/*"SchemeSub.log"*/);
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
                    if (_grbModel_SchemeGenerateSub.ObjVal > 0.0000001)
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
                    if (_grbModel_FlowAssignmentSub.ObjVal > 0.0000001)
                    {
                        ExtremePoint e = new ExtremePoint();
                        foreach (Node n in Data.NodeSet)
                        {
                            double G = _grbModel_FlowAssignmentSub.GetVarByName("g_" + n.ID).X;
                            e.Value.Add(n, G);
                        }
                        foreach (Arc a in Data.ArcSet)
                        {
                            double flow = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID).X +
                                _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.ToNode.ID + "_" + a.FromNode.ID).X;
                            e.TotalCost += flow * Data.FlowFeePerUnit;
                        }
                        e.ID = EPColumnPool.Count;
                        EPColumnPool.Add(e);
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


                /*分支定界的约束*/
                foreach(Node n in CurrentBranchNode.ConstrPool.Keys)
                {
                    GRBVar var = _grbModel_SchemeGenerateSub.GetVarByName("y_" + n.ID);
                    _grbModel_SchemeGenerateSub.AddConstr(var == CurrentBranchNode.ConstrPool[n], "ct_2" + "_" + n.ID);
                }
            }


            GRBEnv _env_FlowAssignmentSub;
            GRBModel _grbModel_FlowAssignmentSub;

            private void OptimizeFASub()
            {
                _env_FlowAssignmentSub = new GRBEnv(/*"FASub.log"*/);
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
                    GRBVar x1 = _grbModel_FlowAssignmentSub.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "x_" + a.FromNode.ID + "_" + a.ToNode.ID);/*正向*/
                    GRBVar x2 = _grbModel_FlowAssignmentSub.AddVar(0.0, GRB.INFINITY, 0.0, GRB.CONTINUOUS, "x_" + a.ToNode.ID + "_" + a.FromNode.ID);/*反向*/
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
                    expr1 += -Data.FlowFeePerUnit * (_grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID)+ _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.ToNode.ID + "_" + a.FromNode.ID));
                }
                _grbModel_FlowAssignmentSub.SetObjective(expr1, GRB.MAXIMIZE);
            }
            private void BuildConst_FA()
            {
                BuildFlowBalanceConst();
                BuildCapacityConst();
                BuildRestrictionConst();
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
                            GRBVar x1 = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
                            exprLeft += x1;
                        }
                        if(a.ToNode==n)
                        {
                            GRBVar x2 = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.ToNode.ID + "_" + a.FromNode.ID);
                            exprLeft += x2;
                        }
                    }
                    exprLeft += n.Demand;
                    GRBLinExpr exprRight = 0;
                    foreach (Arc a in Data.ArcSet)
                    {
                        if (a.ToNode == n)
                        {
                            GRBVar x1 = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
                            exprRight += x1;
                        }
                        if (a.FromNode == n)
                        {
                            GRBVar x2 = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.ToNode.ID + "_" + a.FromNode.ID);
                            exprRight += x2;
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
                    GRBVar x1 = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.FromNode.ID + "_" + a.ToNode.ID);
                    //_grbModel_FlowAssignmentSub.AddConstr(x1 <= a.Capacity, "ct_Cap_" + a.FromNode.ID + "_" + a.ToNode.ID);

                    GRBVar x2 = _grbModel_FlowAssignmentSub.GetVarByName("x_" + a.ToNode.ID + "_" + a.FromNode.ID);
                    _grbModel_FlowAssignmentSub.AddConstr(x1 + x2 <= a.Capacity, "ct_Cap_" + a.ToNode.ID + "_" + a.FromNode.ID);
                }
            }

            private void BuildRestrictionConst()
            {
                foreach(Node n in CurrentBranchNode.ConstrPool.Keys)
                {
                    if (CurrentBranchNode.ConstrPool[n] == 0)
                    {
                        GRBVar g = _grbModel_FlowAssignmentSub.GetVarByName("g_" + n.ID);
                        _grbModel_FlowAssignmentSub.AddConstr(g == 0, "ct_Res_" + n.ID);
                    }
                }
            }
            #endregion
        }




        #region 分支定界
        private void BranchAndBound()
        {
            BranchNode rootBranchNode = new LargeScaleFrmk.CG.BranchNode();
            rootBranchNode.ConstrPool = new Dictionary<LargeScaleFrmk.Node, double>();
            rootBranchNode.ParentNode = null;
            Branching(rootBranchNode);
        }

        private void BuildBranchingTree(BranchNode branchNode)
        {
            Dictionary<Node, double> nodeTestingValue = new Dictionary<Node, double>();
            foreach (Node n in branchNode.ColumnGenerationFrmk.Data.NodeSet)
            {
                double nodeValue = 0;
                foreach (Scheme s in branchNode.ColumnGenerationFrmk.SchemeColumnPool)
                {
                    double var = branchNode.ColumnGenerationFrmk.GetLambda(s);
                    nodeValue += s.Value[n] * var;
                }
                nodeTestingValue.Add(n, nodeValue);
            }
            Node currentNode = null;
            double currentMaxValue = 0;
            foreach(Node n in nodeTestingValue.Keys)
            {
                if (!branchNode.ConstrPool.ContainsKey(n) && nodeTestingValue[n] > currentMaxValue)
                {
                    currentMaxValue = nodeTestingValue[n];
                    currentNode = n;
                }
            }

            branchNode.ColumnGenerationFrmk.DisposeModels();

            if (currentNode == null)
            {
                return;
            }
            //nodeTestingValue = null;
            //if(currentNode.ID=="11")
            //{

            //}

            BranchNode leftBranchNode = new BranchNode();/*创建优势分支*/
            branchNode.LeftChild = leftBranchNode;
            leftBranchNode.ParentNode = branchNode;
            leftBranchNode.ConstrPool = new Dictionary<Node, double>(branchNode.ConstrPool);/*generate constraint pool*/

            BranchNode rightBranchNode = new BranchNode();/*创建劣势分支*/
            branchNode.RightChild = rightBranchNode;
            rightBranchNode.ParentNode = branchNode;
            rightBranchNode.ConstrPool = new Dictionary<Node, double>(branchNode.ConstrPool);/*generate constraint pool*/

            //释放当前节点资源
            //branchNode.ConstrPool.Clear();
            branchNode.ConstrPool = null;
            branchNode.ColumnGenerationFrmk.DisposeModels();
            branchNode.ColumnGenerationFrmk.EPColumnPool = null;
            //branchNode.ColumnGenerationFrmk.SchemeColumnPool.Clear();
            branchNode.ColumnGenerationFrmk = null;
            //System.GC.Collect();
            

            //开始求解优势分支
            if (nodeTestingValue[currentNode]>0.5)
            {
                if(!leftBranchNode.ConstrPool.ContainsKey(currentNode))
                    leftBranchNode.ConstrPool.Add(currentNode, 1);
            }
            else
            {
                if (!leftBranchNode.ConstrPool.ContainsKey(currentNode))
                    leftBranchNode.ConstrPool.Add(currentNode, 0);
            }                                             
            Branching(leftBranchNode);

            //递归返回，做一次上下界监测
            if (branchNode.LB > GlobalUB)
                return;


            //开始求解劣势分支
            if (nodeTestingValue[currentNode] > 0.5)
            {
                rightBranchNode.ConstrPool.Add(currentNode, 0);
            }
            else
            {
                rightBranchNode.ConstrPool.Add(currentNode, 1);
            }
            Branching(rightBranchNode);

            //递归返回，做一次上下界监测
            if (branchNode.LB > GlobalUB)
                return;

            
        }
        private void Branching(BranchNode branchingNode)
        {
            /*CG, Update LB of this branch, cut decision*/
            branchingNode.ColumnGenerationFrmk = new CGFramework();
            branchingNode.ColumnGenerationFrmk.Data = this.Data;
            branchingNode.ColumnGenerationFrmk.CurrentBranchNode = branchingNode;

            branchingNode.LB = branchingNode.ColumnGenerationFrmk.Optimize();
            if (branchingNode.LB == -1)/*infeasible*/
            {
                return;
            }
            if (branchingNode.LB >= GlobalUB)
            {
                branchingNode.ColumnGenerationFrmk.DisposeModels();
                return;
            }

            /*feasibility test, Update global UB, stop criteria*/
            if(FeasibilityTest(branchingNode))
            {
                if(GlobalUB > branchingNode.LB)
                {
                    GlobalUB = branchingNode.LB;
                    OptimalConstrPool = branchingNode.ConstrPool;

                    /*输出可行解*/
                    StreamWriter sw = File.AppendText("NodeLabel.txt");
                    sw.WriteLine("************Get Feasible! ObjVal={0}***************", GlobalUB);
                    sw.Close();
                    /*********/
                }
                branchingNode.ColumnGenerationFrmk.DisposeModels();
            }
            else
            {
                BuildBranchingTree(branchingNode);
            }
        }
        private bool FeasibilityTest(BranchNode branchingNode)
        {
            foreach (Node n in branchingNode.ColumnGenerationFrmk.Data.NodeSet)
            {
                double nodeValue = 0;
                foreach (Scheme s in branchingNode.ColumnGenerationFrmk.SchemeColumnPool)
                {
                    double var = branchingNode.ColumnGenerationFrmk.GetLambda(s);
                    nodeValue += s.Value[n] * var;
                }
                if (nodeValue % 1 > 0.00001)
                    return false;
            }
            return true;
        }
        #endregion

        public void Optimize()
        {
            Data = new LargeScaleFrmk.DataStructure();
            Data.LoadData();

            BranchAndBound();
        }
    }
}

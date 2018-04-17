using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Gurobi;

namespace LargeScaleFrmk
{
    class Benders
    {
        public DataStructure Data = new DataStructure ();
        public List<LazyConstr> ConstrPool = new List<LazyConstr>();
        public SlaveObjCoef SlaveObj;
        public class LazyConstr
        {
            public int ID;
            public Dictionary<Node, double> Alpha = new Dictionary<Node, double> ();
            public Dictionary<Node, double> Beta = new Dictionary<Node, double> ();
            public Dictionary<Arc, double> Gamma = new Dictionary<Arc, double>();
            public string Type;
        }

        public class SlaveObjCoef
        {
            public Dictionary<Node, int> x = new Dictionary<Node, int> ();
        }

        /// <summary>
        /// 主问题
        /// </summary>
        class Master
        {
            public Benders Frmk;
            GRBEnv _env;
            GRBModel _model;
            /// <summary>
            /// 求解主问题
            /// </summary>
            /// <returns></returns>
            public bool Solve()
            {
                _env = new GRBEnv();
                _env.OutputFlag = 0;
                _env.LogToConsole = 0;

                _model = new GRBModel(_env);

                BuildVar();
                BuildConst();

                _model.Write("Benders_Master.lp");
                _model.Optimize();
                if (_model.Status == GRB.Status.OPTIMAL)
                {
                    Frmk.SlaveObj = new SlaveObjCoef();
                    foreach (Node n in Frmk.Data.NodeSet)
                    {
                        double x = _model.GetVarByName("x_" + n.ID).X;
                        Frmk.SlaveObj.x.Add(n, Convert.ToInt32(x));
                    }
                    Output();
                    return true;
                }
                else
                {
                    throw new ApplicationException("没可行解！");
                    return false;
                }
            }
            public void Output()
            {
                Console.WriteLine("---x value---");
                foreach(Node n in Frmk.Data.NodeSet)
                {
                    double x = _model.GetVarByName("x_" + n.ID).X;
                    Console.WriteLine("x_{0}={1}", n.ID, x.ToString());
                }
            }
            public double ObjValue
            {
                get
                {
                    return _model.ObjVal; 
                }
            }
            public void Dispose()
            {
                _model.Dispose();
                _env.Dispose();
            }
            private void BuildVar()
            {
                foreach(Node n in Frmk.Data.NodeSet)
                {
                    GRBVar x = _model.AddVar(0.0, 1.0, Frmk.Data.ServerInstalationFee, GRB.BINARY, "x_" + n.ID);
                }
                GRBVar z = _model.AddVar(0.0, GRB.INFINITY, 1.0, GRB.CONTINUOUS, "z");
                _model.Update();
            }
            private void BuildConst()
            {
                //GRBLinExpr myExpr = 0;
                //foreach (Node n in Frmk.Data.NodeSet)
                //{
                //    myExpr += _model.GetVarByName("x_" + n.ID);
                //}
                //_model.AddConstr(myExpr >= 9, "custom");
                foreach (LazyConstr constr in  Frmk.ConstrPool)
                {
                    GRBLinExpr expr = 0;
                    foreach(Node n in constr.Alpha.Keys)
                    {
                        expr += (-Frmk.Data.M * constr.Alpha[n]) * _model.GetVarByName("x_" + n.ID);
                    }
                    foreach(Node n in constr.Beta.Keys)
                    {
                        expr += n.Demand * constr.Beta[n];
                    }
                    foreach(Arc a in constr.Gamma.Keys)
                    {
                        expr += (-a.Capacity) * constr.Gamma[a];
                    }
                    if(constr.Type=="OPTIMAL")
                    {
                        _model.AddConstr(expr <= _model.GetVarByName("z"), "ct_o_" + constr.ID);
                    }
                    else if(constr.Type=="FEASIBLE")
                    {
                        _model.AddConstr(expr <= 0, "ct_f_" + constr.ID);
                    }
                }
            }
        }

        /// <summary>
        /// 约束生成子问题
        /// </summary>
        class Slave
        {
            public Benders Frmk;
            GRBEnv _env;
            GRBModel _model;
            public bool Solve()
            {
                _env = new GRBEnv();
                _env.OutputFlag = 0;
                _env.LogToConsole = 0;

                _model = new GRBModel(_env);
                _model.Parameters.InfUnbdInfo = 1;

                BuildVar();
                BuildConstr();

                _model.Write("Benders_Slave.lp");
                _model.Optimize();

                if (_model.Status == GRB.Status.OPTIMAL)
                {
                    LazyConstr newConstr = new LargeScaleFrmk.Benders.LazyConstr();
                    foreach(Node n in Frmk.Data.NodeSet)
                    {
                        newConstr.Alpha.Add(n, _model.GetVarByName("alpha_" + n.ID).X);
                        newConstr.Beta.Add(n, _model.GetVarByName("beta_" + n.ID).X);
                    }
                    foreach (Arc a in Frmk.Data.ArcSet)
                    {
                        newConstr.Gamma.Add(a, _model.GetVarByName("gamma_" + a.FromNode.ID + "_" + a.ToNode.ID).X);
                    }
                    newConstr.Type = "OPTIMAL";
                    newConstr.ID = Frmk.ConstrPool.Count;
                    Frmk.ConstrPool.Add(newConstr);

                    Output();
                    return true;
                }
                else if (_model.Status == GRB.Status.UNBOUNDED)
                {
                    LazyConstr newConstr = new LazyConstr();
                    foreach (Node n in Frmk.Data.NodeSet)
                    {
                        newConstr.Alpha.Add(n, _model.GetVarByName("alpha_" + n.ID).Get(GRB.DoubleAttr.UnbdRay));
                        newConstr.Beta.Add(n, _model.GetVarByName("beta_" + n.ID).Get(GRB.DoubleAttr.UnbdRay));
                    }
                    foreach(Arc a in Frmk.Data.ArcSet)
                    {
                        newConstr.Gamma.Add(a, _model.GetVarByName("gamma_" + a.FromNode.ID + "_" + a.ToNode.ID).Get(GRB.DoubleAttr.UnbdRay));
                    }
                    newConstr.Type = "FEASIBLE";
                    newConstr.ID = Frmk.ConstrPool.Count;
                    Frmk.ConstrPool.Add(newConstr);

                    Console.WriteLine("---Unbounded!!---");
                    return true;
                }
                else if (_model.Status == GRB.Status.INFEASIBLE)
                {
                    return false;
                }
                else
                    return false;

            }
            private void Output()
            {
                Console.WriteLine("---g---");
                foreach(Node n in Frmk.Data.NodeSet)
                {
                    double g = _model.GetConstrByName("ct_g_" + n.ID).Pi;
                    Console.WriteLine("g_{0}={1}", n.ID, g.ToString());
                }
                Console.WriteLine("---f---");
                foreach(Arc a in Frmk.Data.ArcSet)
                {
                    double fPlus = _model.GetConstrByName("ct_f+_" + a.FromNode.ID + "_" + a.ToNode.ID).Pi;
                    double fMinus = _model.GetConstrByName("ct_f-_" + a.FromNode.ID + "_" + a.ToNode.ID).Pi;
                    Console.WriteLine("f+_{0}_{1}={2}", a.FromNode.ID, a.ToNode.ID, fPlus.ToString());
                    Console.WriteLine("f-_{0}_{1}={2}", a.FromNode.ID, a.ToNode.ID, fMinus.ToString());
                }
            }
            public double ObjValue
            {
                get
                {
                    if (_model.Status == GRB.Status.UNBOUNDED)
                        return GRB.INFINITY;
                    double masterObjValue = 0;
                    foreach(Node n in Frmk.Data.NodeSet)
                    {
                        masterObjValue += Frmk.SlaveObj.x[n] * Frmk.Data.ServerInstalationFee;
                    }
                    return _model.ObjVal + masterObjValue;
                }
            }
            public void Dispose()
            {
                _model.Dispose();
                _env.Dispose();
            }
            private void BuildVar()
            {
                foreach (Node n in Frmk.Data.NodeSet)
                {
                    GRBVar alpha = _model.AddVar(0.0, GRB.INFINITY, -Frmk.Data.M * Frmk.SlaveObj.x[n], GRB.CONTINUOUS, "alpha_" + n.ID);
                    GRBVar beta = _model.AddVar(-GRB.INFINITY, GRB.INFINITY, n.Demand, GRB.CONTINUOUS, "beta_" + n.ID);
                }
                foreach(Arc a in Frmk.Data.ArcSet)
                {
                    GRBVar gamma = _model.AddVar(0.0, GRB.INFINITY, -a.Capacity, GRB.CONTINUOUS, "gamma_" + a.FromNode.ID + "_" + a.ToNode.ID);
                }
                _model.Update();
                _model.ModelSense = GRB.MAXIMIZE;
            }
            private void BuildConstr()
            {
                foreach(Node n in Frmk.Data.NodeSet)
                {
                    GRBVar alpha = _model.GetVarByName("alpha_" + n.ID);
                    GRBVar beta = _model.GetVarByName("beta_" + n.ID);
                    _model.AddConstr(-alpha + beta <= 0, "ct_g_" + n.ID);
                }
                foreach(Arc a in Frmk.Data.ArcSet)
                {
                    GRBVar fromBeta = _model.GetVarByName("beta_" + a.FromNode.ID);
                    GRBVar toBeta = _model.GetVarByName("beta_" + a.ToNode.ID);
                    GRBVar gamma = _model.GetVarByName("gamma_" + a.FromNode.ID + "_" + a.ToNode.ID);
                    _model.AddConstr(-fromBeta + toBeta - gamma <= Frmk.Data.FlowFeePerUnit, "ct_f+_" + a.FromNode.ID + "_" + a.ToNode.ID);
                    _model.AddConstr(fromBeta - toBeta - gamma <= Frmk.Data.FlowFeePerUnit, "ct_f-_" + a.FromNode.ID + "_" + a.ToNode.ID);
                }
            }
        }

        public void Optimize()
        {
            Data.LoadData();

            do
            {
                Master master = new Master();
                master.Frmk = this;
                if (!master.Solve())
                    throw new ApplicationException("主问题不可行！");

                Slave slave = new Slave();
                slave.Frmk = this;
                if (!slave.Solve())
                    throw new ApplicationException("子问题出错！");

                if (Math.Abs(master.ObjValue - slave.ObjValue) < 0.001)
                {
                    master.Dispose();
                    slave.Dispose();
                    break;
                }
                master.Dispose();
                slave.Dispose();
            }
            while (true);

        }
    }
}

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Gurobi;
using System.IO;

namespace LargeScaleFrmk
{
    public class DataStructure
    {

        public double M = 800.0;
        public double ServerInstalationFee = 100000.0;
        public double FlowFeePerUnit = 1000.0;
        public double ServerCapacity = 800.0;

        public List<Node> NodeSet = new List<Node>();
        public List<Arc> ArcSet = new List<Arc>();
        Node GetNodeByID(string NodeID)
        {
            foreach (Node n in NodeSet)
                if (n.ID == NodeID)
                    return n;
            return null;
        }

        public void LoadData()
        {
            FileStream fs = new FileStream("Node.csv", FileMode.Open);
            StreamReader sr = new StreamReader(fs);
            sr.ReadLine();
            string line = sr.ReadLine();
            while (line != null && line != "")
            {
                string[] data = line.Split(',');
                Node n = new Node();
                n.ID = data[0];
                n.Demand = Convert.ToDouble(data[1]);
                NodeSet.Add(n);
                line = sr.ReadLine();
            }
            sr.Close();
            fs.Close();
            fs = new FileStream("Arc.csv", FileMode.Open);
            sr = new StreamReader(fs);
            sr.ReadLine();
            line = sr.ReadLine();
            while (line != null && line != "")
            {
                string[] data = line.Split(',');
                Arc a = new Arc();
                a.FromNode = GetNodeByID(data[0]);
                a.ToNode = GetNodeByID(data[1]);
                a.FromNode.ArcSet.Add(a);
                a.ToNode.ArcSet.Add(a);
                a.Capacity = Convert.ToDouble(data[2]);
                ArcSet.Add(a);
                line = sr.ReadLine();
            }
            sr.Close();
            sr.Close();
        }
    }

    public class Node
    {
        public List<Arc> ArcSet = new List<Arc>();

        public string ID;
        public double Demand;

        public GRBVar Result_GenerateFlow;
        public GRBVar Result_IsServerLoacationSelected;

        public double GenerateFlow
        {
            get; set;
        }

        public double IsServerLocationSelected
        {
            get; set;
        }

        public void ParseSolution(int i)
        {
            if (i==2 && Result_GenerateFlow as object != null)
                GenerateFlow = Result_GenerateFlow.Get(GRB.DoubleAttr.X);            
            if (i==1 && Result_IsServerLoacationSelected as object != null)
                IsServerLocationSelected = Result_IsServerLoacationSelected.Get(GRB.DoubleAttr.X);
        }

    }
    public class Arc
    {
        public Node FromNode;
        public Node ToNode;

        public double Capacity;

        public GRBVar Result_FlowF;
        public GRBVar Result_FlowR;

        public double FlowF { get; set; }
        public double FlowR { get; set; }

        public void ParseSolution()
        {
            if (Result_FlowF as object != null)
                FlowF = Result_FlowF.Get(GRB.DoubleAttr.X);
            if (Result_FlowR as object != null)
                FlowR = Result_FlowR.Get(GRB.DoubleAttr.X);
        }
    }
}

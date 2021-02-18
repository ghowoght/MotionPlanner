using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionPlanner
{
    public class KDNode : IComparable<KDNode>
    {
        public List<double> data = null;
        public int split;
        public KDNode left = null;
        public KDNode right = null;
        public KDNode parent = null;

        public double dist = 0;

        private int dim;

        public KDNode() { }

        public KDNode(List<double> data)
        {
            this.data = data;
        }

        public KDNode(List<double> data, int split, KDNode left, KDNode right)
        {
            this.data = data;
            this.split = split;
            this.left = left;
            this.right = right;
            this.dim = data.Count;
        }

        public override string ToString()
        {
            StringBuilder sb = new StringBuilder();
            foreach(double t in data)
                sb.Append(t + " ");
            return sb.ToString();
        }

        public int CompareTo(KDNode other)
        {
            return this.dist < other.dist ? 1 : (this.dist == other.dist ? 0 : -1);
        }
    }

    public class KDTree
    {
        public KDNode root = null;

        public KDTree() { }
        public KDTree(List<List<double>> dataArray)
        {
            root = make_tree(dataArray, 0);
        }

        /// <summary>
        /// 先根遍历
        /// </summary>
        /// <param name="node"></param>
        public static void preOrderTraveral(KDNode node)
        {
            if (node == null)
                return;
            Console.WriteLine(node.ToString() + " " + node.split);
            preOrderTraveral(node.left);
            preOrderTraveral(node.right);
        }

        private void FastSelect(List<List<double>> dataArray, int split)
        {
            List<List<List<double>>> result = new List<List<List<double>>>();


            int middle = dataArray.Count / 2; // 中位数

            for(int i = 1; i < dataArray.Count; i++)
            {
                List<double> point = dataArray[i];
            }
        }
        private KDNode make_tree(List<List<double>> dataArray, int split)
        {
            if (dataArray.Count == 0)
                return null;

            // => lambda表达式，实现对指定列的排序(默认升序)
            dataArray.Sort((List<double> x, List<double> y) => { return x[split].CompareTo(y[split]); });

            int middle = dataArray.Count / 2; // 中位数

            KDNode node = new KDNode(   dataArray[middle],
                                        split,
                                        make_tree(dataArray.GetRange(0, middle),                                (split + 1) % dataArray[middle].Count),
                                        make_tree(dataArray.GetRange(middle + 1, dataArray.Count - middle - 1), (split + 1) % dataArray[middle].Count));
            return node;
        }

        public void Add(List<double> point)
        {
            KDNode nodeToAdd = new KDNode(point);

            if(root == null)
            {
                root = new KDNode(point);
            }
            else
            {
                
                KDNode near = GetNearest(point);
                while (true)
                {
                    int split = near.split;
                    if (point[split] <= near.data[split])
                    {
                        if (near.left == null)
                        {
                            near.left = nodeToAdd;
                            break;
                        }
                        else
                        {
                            near = near.left;
                        }
                    }
                    else
                    {
                        if (near.right == null)
                        {
                            near.right = nodeToAdd;
                            break;
                        }
                        else
                        {
                            near = near.right;
                        }
                    }
                }

            }
        }
        /// <summary>
        /// 计算两个点的欧氏距离
        /// </summary>
        /// <param name="p1"></param>
        /// <param name="p2"></param>
        /// <returns></returns>
        static public double GetDistance(List<double> p1, List<double> p2)
        {
            double dist = 0;
            if (p1.Count != p2.Count)
                return dist;
            for (int i = 0; i < p1.Count; i++)
                dist += Math.Pow(p1[i] - p2[i], 2);
            return Math.Sqrt(dist);
        }
        /// <summary>
        /// 获取最近邻
        /// </summary>
        /// <param name="point">查询点</param>
        /// <returns></returns>
        public KDNode GetNearest(List<double> point)
        {
            KDNode best = root;
            double best_dist = GetDistance(point, best.data);
            KDNode next = root;
            Stack<KDNode> nodes = new Stack<KDNode>();
            // 二分查找，直到找到一个叶子节点，同时记录查找过程中的最近邻
            while (next != null)
            {
                nodes.Push(next);
                double next_dist = GetDistance(point, next.data);
                if (next_dist < best_dist)
                {
                    best = next;
                    best_dist = next_dist;
                }

                if (point[next.split] <= next.data[next.split])
                    next = next.left;
                else
                    next = next.right;
            }
            // 回溯，依次判断与之前节点的分割轴是否有相交
            while (nodes.Count != 0)
            {
                next = nodes.Pop();
                if (Math.Abs(point[next.split] - next.data[next.split]) < best_dist) // 以查询点为球心的超球面与分割超平面相交
                {
                    // 进入另一个分支
                    if (point[next.split] <= next.data[next.split])
                        next = next.right; 
                    else 
                        next = next.left;
                    // 将另一个的分支所有节点加入nodes
                    while (next != null)
                    {
                        nodes.Push(next);
                        double next_dist = GetDistance(point, next.data);
                        if (next_dist < best_dist)
                        {
                            best = next;
                            best_dist = next_dist;
                        }

                        if (point[next.split] <= next.data[next.split])
                            next = next.left;
                        else
                            next = next.right;
                    }
                }
            }
            return best;
        }
        /// <summary>
        /// 获取K个最近邻
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public List<KDNode> GetKNearest(List<double> point, int K)
        {

            PriorityQueue<KDNode> bests = new PriorityQueue<KDNode>();

            KDNode best = root;
            best.dist = GetDistance(point, best.data);
            bests.Push(best);
            if (bests.Count > K)
                bests.Pop();

            KDNode next = root;
            Stack<KDNode> nodes = new Stack<KDNode>();
            // 二分查找，直到找到一个叶子节点，同时记录查找过程中的最近邻
            while (next != null)
            {
                nodes.Push(next);
                double next_dist = GetDistance(point, next.data);
                if (next_dist < bests.Top().dist)
                {
                    best = next;
                    best.dist = next_dist;

                    bests.Push(best);
                    if (bests.Count > K)
                        bests.Pop();
                }

                if (point[next.split] <= next.data[next.split])
                    next = next.left;
                else
                    next = next.right;
            }
            // 回溯，依次判断与之前节点的分割轴是否有相交
            while (nodes.Count != 0)
            {
                next = nodes.Pop();
                if (Math.Abs(point[next.split] - next.data[next.split]) < bests.Top().dist) // 以查询点为球心的超球面与分割超平面相交
                {
                    // 进入另一个分支
                    if (point[next.split] <= next.data[next.split])
                        next = next.right;
                    else
                        next = next.left;
                    // 将另一个的分支所有节点加入nodes
                    while (next != null)
                    {
                        nodes.Push(next);
                        double next_dist = GetDistance(point, next.data);
                        if (next_dist < bests.Top().dist)
                        {
                            best = next;
                            best.dist = next_dist;

                            bests.Push(best);
                            if (bests.Count > K)
                                bests.Pop();
                        }

                        if (point[next.split] <= next.data[next.split])
                            next = next.left;
                        else
                            next = next.right;
                    }
                }
            }
            return bests.datas;

        }

        /// <summary>
        /// 获取输入节点周围指定范围内的节点
        /// </summary>
        /// <param name="point"></param>
        /// <param name="R">范围半径</param>
        /// <returns></returns>
        public List<KDNode> GetRange(List<double> point, double R)
        {
            List<KDNode> bests = new List<KDNode>();

            KDNode best = root;
            best.dist = GetDistance(point, best.data);

            KDNode next = root;
            Stack<KDNode> nodes = new Stack<KDNode>();
            // 二分查找
            while (next != null)
            {
                nodes.Push(next);
                double next_dist = GetDistance(point, next.data);
                if (next_dist < R)
                {
                    best = next;
                    best.dist = next_dist;

                    bests.Add(best);
                }

                if (point[next.split] <= next.data[next.split])
                    next = next.left;
                else
                    next = next.right;
            }
            // 回溯
            while (nodes.Count != 0)
            {
                next = nodes.Pop();
                if (Math.Abs(point[next.split] - next.data[next.split]) < R) // 以查询点为球心、R为半径的超球面与分割超平面相交
                {
                    // 进入另一个分支
                    if (point[next.split] <= next.data[next.split])
                        next = next.right;
                    else
                        next = next.left;
                    // 将另一个的分支所有节点加入nodes
                    while (next != null)
                    {
                        nodes.Push(next);
                        double next_dist = GetDistance(point, next.data);
                        if (next_dist < R)
                        {
                            best = next;
                            best.dist = next_dist;

                            bests.Add(best);
                        }

                        if (point[next.split] <= next.data[next.split])
                            next = next.left;
                        else
                            next = next.right;
                    }
                }
            }
            return bests;

        }

    }
}

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MotionPlanner
{
    public class KDNode : IComparable<KDNode>
    {
        public bool valid = true;
        public int x = 0;
        public int y = 0;
        public double cost = 0;
        public List<KDNode> neighbor = new List<KDNode>();
        public KDNode front = null;
        public KDNode() { }
        public KDNode(int x, int y)
        {
            this.x = x;
            this.y = y;
            this.data = new List<double> { (double)x, (double)y };
        }
        public KDNode(List<double> data)
        {
            this.data = data;
            this.x = (int)data[0];
            this.y = (int)data[1];
        }

        public void Remove(KDNode node)
        {
            for (int i = 0; i < neighbor.Count; i++)
            {
                if (neighbor[i].x == node.x && neighbor[i].y == node.y)
                {
                    neighbor.RemoveAt(i);
                }
            }
        }

        /// <summary>
        /// 将Node转换为Point形式
        /// </summary>
        /// <returns>Point</returns>
        public Point Node2Point()
        {
            return new Point(x, y);
        }

        public int CompareTo(KDNode other)
        {
            return this.cost < other.cost ? -1 : (this.cost == other.cost ? 0 : 1);
        }

        public bool isEqual(KDNode other) // 判断输入结点和本结点是否相同
        {
            if (this.x == other.x && this.y == other.y)
            {
                return true;
            }
            return false;
        }
        public List<double> data = null;
        public int split;
        public KDNode left = null;
        public KDNode right = null;
        public KDNode parent = null;

        public double dist = 0;

        private int dim;

        //public KDNode() { }

        //public KDNode(List<double> data)
        //{
        //    this.data = data;
        //}

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

        //public int CompareTo(KDNode other)
        //{
        //    return this.dist < other.dist ? 1 : (this.dist == other.dist ? 0 : -1);
        //}
    }

    public class KDTree
    {
        public List<KDNode> kdnodes = new List<KDNode>();
        public KDNode root = null;

        public KDTree() { }
        public KDTree(List<List<double>> dataArray)
        {
            root = make_tree(dataArray, 0);
        }
        public KDTree(List<KDNode> kdnodes)
        {
            root = make_tree(kdnodes, 0);
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


        private KDNode make_tree(List<List<double>> dataArray, int split)
        {
            if (dataArray.Count == 0)
                return null;

            // => lambda表达式，实现对指定列的排序(默认升序)
            dataArray.Sort((List<double> x, List<double> y) => { return x[split].CompareTo(y[split]); });

            int middle = dataArray.Count / 2; // 中位数

            KDNode node = new KDNode(dataArray[middle],
                                        split,
                                        make_tree(dataArray.GetRange(0, middle), (split + 1) % dataArray[middle].Count),
                                        make_tree(dataArray.GetRange(middle + 1, dataArray.Count - middle - 1), (split + 1) % dataArray[middle].Count));
            kdnodes.Add(node);
            return node;
        }

        private KDNode make_tree(List<KDNode> kdnodes, int split)
        {
            if (kdnodes.Count == 0)
                return null;

            // => lambda表达式，实现对指定列的排序(默认升序)
            kdnodes.Sort((KDNode x, KDNode y) => { return x.data[split].CompareTo(y.data[split]); });

            int middle = kdnodes.Count / 2; // 中位数

            KDNode node = kdnodes[middle];
            node.split = split;
            node.left = make_tree(kdnodes.GetRange(0, middle), (split + 1) % kdnodes[middle].data.Count);
            node.right = make_tree(kdnodes.GetRange(middle + 1, kdnodes.Count - middle - 1), (split + 1) % kdnodes[middle].data.Count);
            //new KDNode(kdnodes[middle].data,
            //                            split,
            //                            make_tree(kdnodes.GetRange(0, middle), (split + 1) % kdnodes[middle].data.Count),
            //                            make_tree(kdnodes.GetRange(middle + 1, kdnodes.Count - middle - 1), (split + 1) % kdnodes[middle].data.Count));
            
            this.kdnodes.Add(node);
            return node;
        }

        public void Add(List<double> point)
        {
            KDNode nodeToAdd = new KDNode(point);
            kdnodes.Add(nodeToAdd);
            if (root == null)
            {
                root = nodeToAdd;// new KDNode(point);

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

        public void Add(KDNode nodeToAdd)
        {
            kdnodes.Add(nodeToAdd);
            if (root == null)
            {
                root = nodeToAdd;// new KDNode(point);

            }
            else
            {

                KDNode near = GetNearest(nodeToAdd.data);
                while (true)
                {
                    int split = near.split;
                    if (nodeToAdd.data[split] <= near.data[split])
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
                //cnt_neighbors++;
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
        public int cnt_neighbors = 0;
        public List<KDNode> GetNeighbors_(KDNode branch,
                                            List<double> point,
                                            double R)
        {
            
            //if (branch == null)
            //    return new List<KDNode>();
            //cnt_neighbors++;
            double d = GetDistance(point, branch.data);
            double r2 = R * R;

            List<KDNode> neighbors = new List<KDNode>();

            if (d <= R)
                neighbors.Add(branch);
            KDNode section, other;
            if (point[branch.split] <= branch.data[branch.split])
            {
                section = branch.left;
                other = branch.right;
            }
            else
            {
                section = branch.right;
                other = branch.left;
            }
            if (section != null)
                neighbors.AddRange(GetNeighbors_(section, point, R));
            if ((branch.data[branch.split] - point[branch.split])
                * (branch.data[branch.split] - point[branch.split])
                < r2)
            {
                if (other != null)
                    neighbors.AddRange(GetNeighbors_(other, point, R));
            }

            return neighbors;
        }
        public List<KDNode> GetNeighbors(List<double> point, double R)
        {
            List<KDNode> neighbors = GetNeighbors_(root, point, R);
            return neighbors;
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
                    if (next_dist < best.dist)
                    {
                        best = next;
                        best.dist = next_dist;
                    }

                    bests.Add(next);
                }

                if (point[next.split] <= next.data[next.split])
                    next = next.left;
                else
                    next = next.right;
            }
            // 回溯
            while (nodes.Count != 0)
            {
                //Console.WriteLine("node_cnt: " + nodes.Count);
                //cnt_neighbors++;
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

        public static void Test()
        {
            List<List<double>> dataArray = new List<List<double>>();
            KDTree tree = new KDTree();

            Random rnd = new Random(12);

            const int PointNum = 1000;

            Stopwatch sw = new Stopwatch();
            sw.Start();

            for (int i = 0; i < PointNum; i++)
            {
                dataArray.Add(new List<double>());
                dataArray[i].Add(rnd.NextDouble() * 100);
                dataArray[i].Add(rnd.NextDouble() * 100);
                //dataArray[i].Add(rnd.NextDouble() * 100);
                tree.Add(dataArray[i]);
            }
            //tree = new KDTree(tree.kdnodes);
            //tree = new KDTree(dataArray);
            //Console.WriteLine("节点数：" + tree.kdnodes.Count());
            sw.Stop();
            Console.WriteLine("KD-Tree构建耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");

            List<double> point = new List<double> { rnd.NextDouble() * 100,
                                                    //rnd.NextDouble() * 100,
                                                    rnd.NextDouble() * 100};

            sw.Restart();
            //tree.Add(new List<double>{  rnd.NextDouble() * 100,
            //                            //rnd.NextDouble() * 100,
            //                            rnd.NextDouble() * 100});
            KDNode node = tree.GetNearest(point);
            sw.Stop();

            Console.WriteLine("KD-Tree最近邻查找");
            Console.WriteLine("查找点: " + new KDNode(point).ToString());
            Console.WriteLine("最近邻: " + node.ToString());
            Console.WriteLine("距离  : " + KDTree.GetDistance(point, node.data));
            Console.WriteLine("耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");

            sw.Restart();
            KDNode nearest = new KDNode();
            double minDist = KDTree.GetDistance(point, dataArray[0]);
            for (int i = 1; i < dataArray.Count; i++)
            {
                if (minDist > KDTree.GetDistance(point, dataArray[i]))
                {
                    nearest.data = dataArray[i];
                    minDist = KDTree.GetDistance(point, dataArray[i]);
                }
            }
            sw.Stop();
            Console.WriteLine("暴力搜索");
            Console.WriteLine("查找点: " + new KDNode(point).ToString());
            Console.WriteLine("最近邻: " + nearest.ToString());
            Console.WriteLine("距离  : " + KDTree.GetDistance(point, nearest.data));
            Console.WriteLine("耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");
            Console.WriteLine("-------------------");

            // K个最近邻搜索

            tree = new KDTree(tree.kdnodes);

            // 搜索距离输入节点指定范围内的所有节点
            Console.WriteLine("搜索距离输入节点指定范围内的所有节点");

            Console.WriteLine("查找点: " + new KDNode(point).ToString());
            double R = 30;
            List<KDNode> nears0 = tree.GetRange(point, R);
            sw.Restart();
            List<KDNode>  nears = tree.GetRange(point, R);
            //List<KDNode> nears = tree.GetNeighbors(point, R);
            //List<KDNode> nears = tree.GetKNearest(point, 50);
            sw.Stop();
            Console.WriteLine("KD-Tree搜索");
            Console.WriteLine("耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");
            Console.WriteLine("找到  : " + nears.Count + "个");
            Console.WriteLine(tree.cnt_neighbors);
            //foreach (KDNode node in nears)
            //{
            //    Console.WriteLine("最近邻: " + node.ToString() + " " + KDTree.GetDistance(point2, node.data));
            //}

            List<List<double>> nears2 = new List<List<double>>();

            sw.Restart();
            for (int i = 1; i < dataArray.Count; i++)
            {
                if (R > KDTree.GetDistance(point, dataArray[i]))
                {
                    nears2.Add(dataArray[i]);
                }
            }
            sw.Stop();
            Console.WriteLine("暴力搜索");
            Console.WriteLine("耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");
            Console.WriteLine("找到  : " + nears2.Count + "个");
            //foreach (List<double> node in nears2)
            //{
            //    Console.WriteLine(KDTree.GetDistance(point2, node));
            //}


            return;
            //KDTree.preOrderTraveral(tree.root);
        }

    }
}

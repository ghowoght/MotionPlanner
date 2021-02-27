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
    class InformedRRT : SamplingPlanner
    {
        int ITER = 1000;                     // 迭代次数 / 采样点数
        const double MAX_DISTANCE = 15;             // 两个节点建立连接的最大距离
        const double OptimizationR = 30;            // 优化半径
        public Graph samplesGraph = new Graph();    // 采样后的样点图

        //Hyperellipsoid hyperellipsoid;              // 超椭圆参数

        KDTree kdtree = new KDTree();

        public class Hyperellipsoid
        {
            public double c_max = 0;
            public double c_min = 0;

            public double[,] rotationMatrix = new double[2, 2];

            public KDNode centre = new KDNode();

            public Hyperellipsoid() { }
            public Hyperellipsoid(KDNode origin, KDNode goal)
            {
                c_min = GetEuclideanDistance(goal, origin);
                centre = new KDNode((origin.x + goal.x) / 2, (origin.y + goal.y) / 2);
                double theta = -Math.Atan2(goal.y - origin.y, goal.x - origin.x);
                rotationMatrix[0, 0] = Math.Cos(theta);
                rotationMatrix[0, 1] = Math.Sin(theta);
                rotationMatrix[1, 0] = -Math.Sin(theta);
                rotationMatrix[1, 1] = Math.Cos(theta);
            }

        }

        public InformedRRT(GridMap map)
        {
            this.map = map;
            //map.graph = samplesGraph;
            map.kdtree = kdtree;
        }

        public KDNode RotationToWorldFrame(KDNode sample, Hyperellipsoid hyperellipsoid)
        {
            KDNode sample_new = new KDNode((int)(hyperellipsoid.rotationMatrix[0, 0] * sample.x + hyperellipsoid.rotationMatrix[0, 1] * sample.y + hyperellipsoid.centre.x),
                                        (int)(hyperellipsoid.rotationMatrix[1, 0] * sample.x + hyperellipsoid.rotationMatrix[1, 1] * sample.y + hyperellipsoid.centre.y));
            sample_new.x = sample_new.x < 0 ? 0 : sample_new.x;
            sample_new.x = sample_new.x >= map.Height ? (map.Height - 1) : sample_new.x;
            sample_new.y = sample_new.y < 0 ? 0 : sample_new.y;
            sample_new.y = sample_new.y >= map.Width ? (map.Width - 1) : sample_new.y;

            return sample_new;
        }

        public KDNode GetSample(Hyperellipsoid hyperellipsoid, double c_max, Random rnd)
        {
            KDNode sample = new KDNode(rnd.Next(0, map.Height), rnd.Next(0, map.Width));
            if (c_max == double.MaxValue)
            {
                while (map.map[sample.x][sample.y] == (int)GridMap.MapStatus.Occupied)
                {
                    sample = new KDNode(rnd.Next(0, map.Height), rnd.Next(0, map.Width));
                }
                return sample;
            }
            double x_max = c_max / 2; // 长轴

            double c = hyperellipsoid.c_min / 2.0; // 焦点坐标
            double y_max = Math.Sqrt(x_max * x_max + c * c); // 短轴

            do
            {
                KDNode sample0 = new KDNode(rnd.Next((int)-x_max, (int)x_max), rnd.Next((int)-y_max, (int)y_max));
                // 判断采样点是否在椭圆内
                // 如果采样点到两焦点的距离之和小于2a，则采样点在椭圆内
                while (GetEuclideanDistance(sample0, new KDNode((int)-c, 0)) + GetEuclideanDistance(sample0, new KDNode((int)c, 0)) > 2 * x_max)
                {
                    sample0 = new KDNode(rnd.Next((int)-x_max, (int)x_max),
                                        rnd.Next((int)-y_max, (int)y_max));
                }

                sample = RotationToWorldFrame(sample0, hyperellipsoid);
            } while (map.map[sample.x][sample.y] == (int)GridMap.MapStatus.Occupied);
            return sample;
        }

        public void Search()
        {
            Stopwatch sw = new Stopwatch();
            KDNode origin = new KDNode(map.origin.X, map.origin.Y); // 起点
            KDNode goal = new KDNode(map.goal.X, map.goal.Y);       // 终点

            // 初始化超椭圆参数
            Hyperellipsoid hyperellipsoid = new Hyperellipsoid(origin, goal);
            // 将起点加入采样图
            kdtree.Add(origin);

            Random rnd = new Random();

            double c_best = double.MaxValue;
            for (int iter = 0; iter < ITER; iter++)
            {
                //if (c_best < 263)
                //    break;
                // 保存原始采样点
                KDNode sample0 = GetSample(hyperellipsoid, c_best, rnd); 

                // 初始化采样点为原始采样点
                KDNode sample = new KDNode(sample0.x, sample0.y);
                // 最近邻搜索
                //sw.Start();
                sample.front = kdtree.GetNearest(sample.data);
                double distance_near = GetEuclideanDistance(sample.front, sample);
                //sw.Stop();
                if (distance_near > MAX_DISTANCE)
                {
                    // 如果原始采样点与当前节点距离大于最大距离
                    // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                    // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                    double theta = Math.Atan2(sample0.y - sample.front.y, sample0.x - sample.front.x); // 倾角
                    sample.x = sample.front.x + (int)(MAX_DISTANCE * Math.Cos(theta));
                    sample.y = sample.front.y + (int)(MAX_DISTANCE * Math.Sin(theta));
                    sample.data = new List<double> { (double)sample.x, (double)sample.y };
                }
                if (!isCollision(sample.front, sample)) // 如果采样点和已知样点图间存在一条无碰撞路径
                {
                    //samplesGraph.kdnodes.Add(sample);
                    kdtree.Add(sample);
                    sample.front.neighbor.Add(sample);
                    sample.cost = sample.front.cost + GetEuclideanDistance(sample, sample.front); // 计算代价

                    sw.Start();
                    // 重连 Rewire
                    double R = OptimizationR;
                    //double R = 50 * Math.Sqrt(Math.Log(samplesGraph.kdnodes.Count()) / (double)samplesGraph.kdnodes.Count());
                    List<KDNode> nearests = kdtree.GetRange(sample.data, R);
                    foreach (KDNode n in nearests) // 遍历所有潜在父节点
                    //foreach (KDNode n in kdtree.kdnodes) // 遍历所有潜在父节点
                    {
                        double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                        if (cost < sample.cost) // 如果新路径代价更小
                        {
                            if (!isCollision(sample, n, R)) // 碰撞检测通过
                            {
                                sample.front.Remove(sample); // 将之前的边删掉
                                sample.cost = cost; // 更新代价
                                sample.front = n; // 更新路径
                                sample.front.neighbor.Add(sample); // 添加新边
                            }
                        }

                    }
                    sw.Stop();
                    //Console.WriteLine("Rewire Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
                    //sw.Reset();

                    // 到达终点附近，回溯得到路径
                    if (!isCollision(sample, goal, MAX_DISTANCE))
                    {
                        double c_new = sample.cost + GetEuclideanDistance(goal, sample);

                        if (c_new < c_best) // 新路径的长度更小
                        {
                            // 回溯路径
                            List<Point> path = new List<Point>();
                            path.Add(map.goal);
                            while (sample.front != null)
                            {
                                path.Add(new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            path.Add(map.origin);

                            map.road = path;
                            c_best = c_new;

                            // 根据路径长度计算迭代次数
                            ITER = (int)c_best * 15;

                            // 剔除超椭圆外的样点
                            List<KDNode> kdnodes = new List<KDNode>();
                            for (int i = 0; i < kdtree.kdnodes.Count(); i++)
                            {
                                KDNode kdnode = kdtree.kdnodes[i];
                                if (GetEuclideanDistance(kdnode, origin)
                                    + GetEuclideanDistance(kdnode, goal) < c_best)
                                {
                                    kdnodes.Add(kdnode);
                                }
                            }
                            //sw.Start();
                            kdtree = new KDTree(kdnodes);
                            map.kdtree = kdtree;
                            //sw.Stop();
                            
                        }
                        Console.WriteLine("iter: " + iter + " c_best: " + c_best + " " + kdtree.kdnodes.Count());
                        //Thread.Sleep(200);
                    }
                }
            }

            Console.WriteLine(kdtree.kdnodes.Count() + " Rewire Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
            Thread.Sleep(200);
            map.searchFlag = 1;
        }

        public class Path
        {
            public double cost = 0;
            public KDNode origin;
            public KDNode goal;
            public List<KDNode> path = new List<KDNode>();
            public List<Point> road = new List<Point>(); // 用于画板显示
            public Path() { }
            public Path(KDNode origin, KDNode goal, List<KDNode> path)
            {
                this.origin = origin;
                this.goal = goal;
                this.path = path;
            }

            /// <summary>
            /// 合并路径
            /// </summary>
            /// <param name="other">另一条路径</param>
            /// <returns>新路径</returns>
            public List<KDNode> Combine(Path other)
            {
                List<KDNode> new_path = new List<KDNode>();
                // 寻找两条路径交汇节点的索引
                int combineIndex;
                for (combineIndex = 0; combineIndex < path.Count && combineIndex < other.path.Count; combineIndex++)
                    if (!path[combineIndex].isEqual(other.path[combineIndex]))
                        break;
                // 将所有节点按顺序整合到路径列表上
                for (int i = combineIndex; i < path.Count; i++)
                {
                    KDNode nodeToAdd = new KDNode(path[i].data);
                    new_path.Add(nodeToAdd);
                }
                new_path.Reverse();
                new_path.Add(new KDNode(path[combineIndex - 1].data));
                for (int i = combineIndex; i < other.path.Count; i++)
                {
                    KDNode nodeToAdd = new KDNode(other.path[i].data);
                    new_path.Add(nodeToAdd);
                }
                // 重新建立节点间的连接关系
                for (int i = 0; i < new_path.Count - 1; i++)
                {
                    new_path[i + 1].front = new_path[i];
                    new_path[i + 1].cost = new_path[i + 1].front.cost + GetEuclideanDistance(new_path[i + 1].front, new_path[i + 1]);
                }

                return new_path;
            }
        }

        public void Search(object goals1)
        {
            
            List<KDNode> goals = (List<KDNode>)goals1;
            //map.goals = goals;
            //Thread.Sleep(10000);
            KDNode centre = new KDNode( goals.Sum((KDNode node) => { return node.x; }) / goals.Count,
                                        goals.Sum((KDNode node) => { return node.y; }) / goals.Count);

            KDNode origin = goals.Min((KDNode n) => { KDNode n0 = new KDNode(n.x, n.y); n0.cost = GetEuclideanDistance(n, centre); return n0; });
            origin = new KDNode(origin.data);

            // 将origin移到目标点列表的开头
            goals.Remove(goals.Where(n => n.isEqual(origin)).ToList<KDNode>()[0]);
            goals.Insert(0, origin);

            // 初始化超椭圆参数
            Hyperellipsoid hyperellipsoid = new Hyperellipsoid();

            List<KDNode> goals0 = new List<KDNode>();// = new List<KDNode>(goals);
            goals0.Add(origin);
            map.goals = goals0;
            List<Path> pathsList = new List<Path>();

            Stopwatch sw = new Stopwatch();

            // 将起点加入采样图
            kdtree.Add(origin);

            Random rnd = new Random();

            while (goals.Count != 1)
            {
                //if (c_best < 263)
                //    break;
                // 保存原始采样点
                KDNode sample0 = GetSample(hyperellipsoid, double.MaxValue, rnd);

                // 初始化采样点为原始采样点
                KDNode sample = new KDNode(sample0.x, sample0.y);
                // 最近邻搜索
                //sw.Start();
                //sample.front = kdtree.GetNearest(sample.data);
                sample.front = kdtree.kdnodes[0];
                double distance_near = GetEuclideanDistance(sample.front, sample);
                for (int i = 1; i < kdtree.kdnodes.Count; i++)
                {
                    double cost = GetEuclideanDistance(sample, kdtree.kdnodes[i]);
                    if (cost < distance_near)
                    {
                        sample.front = kdtree.kdnodes[i];
                        distance_near = cost;
                    }
                }
                
                //sw.Stop();
                if (distance_near > MAX_DISTANCE)
                {
                    // 如果原始采样点与当前节点距离大于最大距离
                    // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                    // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                    double theta = Math.Atan2(sample0.y - sample.front.y, sample0.x - sample.front.x); // 倾角
                    sample.x = sample.front.x + (int)(MAX_DISTANCE * Math.Cos(theta));
                    sample.y = sample.front.y + (int)(MAX_DISTANCE * Math.Sin(theta));
                    sample.data = new List<double> { (double)sample.x, (double)sample.y };
                }
                if (!isCollision(sample.front, sample)) // 如果采样点和已知样点图间存在一条无碰撞路径
                {
                    //Console.WriteLine(sample.x + " " + sample.y);
                    //samplesGraph.kdnodes.Add(sample);
                    kdtree.Add(sample);
                    sample.front.neighbor.Add(sample);
                    sample.cost = sample.front.cost + GetEuclideanDistance(sample, sample.front); // 计算代价

                    sw.Start();
                    // 重连 Rewire
                    double R = OptimizationR;
                    //double R = 50 * Math.Sqrt(Math.Log(samplesGraph.kdnodes.Count()) / (double)samplesGraph.kdnodes.Count());
                    List<KDNode> nearests = kdtree.GetRange(sample.data, R);
                    //foreach (KDNode n in nearests) // 遍历所有潜在父节点
                    foreach (KDNode n in kdtree.kdnodes) // 遍历所有潜在父节点
                    {
                        double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                        if (cost < sample.cost) // 如果新路径代价更小
                        {
                            if (!isCollision(sample, n, R)) // 碰撞检测通过
                            {
                                sample.front.Remove(sample); // 将之前的边删掉
                                sample.cost = cost; // 更新代价
                                sample.front = n; // 更新路径
                                sample.front.neighbor.Add(sample); // 添加新边
                            }
                        }
                        else
                        {
                            cost = sample.cost + GetEuclideanDistance(sample, n); // 计算代价
                            if (cost < n.cost) // 如果新路径代价更小
                            {
                                if (!isCollision(sample, n, R)) // 碰撞检测通过
                                {
                                    //n.front.Remove(n); // 将之前的边删掉
                                    n.cost = cost; // 更新代价
                                    n.front = sample; // 更新路径
                                                      //n.front.neighbor.Add(n); // 添加新边
                                }
                            }
                        }

                    }
                    sw.Stop();


                    for (int i = 1; i < goals.Count; i++)
                    {
                        // 到达终点附近，回溯得到路径
                        if (!isCollision(sample, goals[i], OptimizationR))
                        {
                            double c_new = sample.cost + GetEuclideanDistance(goals[i], sample);
                            goals[i].front = sample;
                            goals[i].cost = c_new;

                            Path path = new Path();
                            path.cost = c_new;
                            path.path.Add(goals[i]);
                            path.goal = goals[i];

                            // 回溯路径
                            List<Point> road = new List<Point>();

                            road.Add(goals[i].Node2Point());
                            while (sample.front != null)
                            {

                                path.path.Add(sample);
                                road.Add(new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            road.Add(sample.Node2Point());
                            path.path.Add(origin);
                            path.origin = sample;
                            path.road = road;

                            path.path.Reverse();
                            pathsList.Add(path);

                            map.roads.Add(road);
                            goals0.Add(goals[i]);
                            //Thread.Sleep(500);

                            goals.RemoveAt(i);
                            Console.WriteLine("find!" + i);

                            break;
                        }
                    }
                }
            }
            //Thread.Sleep(2000);
            //Console.WriteLine(kdtree.kdnodes.Count() + " Rewire Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
            //Thread.Sleep(2000);
            map.roads = new List<List<Point>>();

            // 从搜索树中提取所有路径
            for (int i = 0; i < goals0.Count - 1; i++)
            {
                for (int j = i + 1; j < goals0.Count - 1; j++)
                {
                    Path new_path = new Path(new KDNode(pathsList[i].goal.data),
                        new KDNode(pathsList[j].goal.data),
                        pathsList[i].Combine(pathsList[j]));
                    new_path.cost = new_path.path[new_path.path.Count - 1].cost;
                    pathsList.Add(new_path);
                }
            }
            // 单路径优化
            sw.Restart();
            Parallel.For(0, pathsList.Count, i =>
            {
                Console.WriteLine("Fingding: " + i);
                Search(pathsList[i]);
                Console.WriteLine("Found!" + i);
            });
            foreach (Path path in pathsList)
            //{
            //    Console.WriteLine("Fingding: ");
            //    Search(path);
            //    Console.WriteLine("Found!");
            //}
            sw.Stop();
            Console.WriteLine("Optimaze Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
            List<List<double>> costGraph = new List<List<double>>();
            int index = 0;
            for (int i = 0; i < goals0.Count; i++)
            {
                costGraph.Add(new List<double>());
                for (int j = 0; j < goals0.Count; j++)
                {
                    if (j > i)
                        costGraph[i].Add(pathsList[index++].cost);
                    else
                        costGraph[i].Add(0);
                }
            }
            for (int i = 0; i < goals0.Count; i++)
            {
                for (int j = 0; j < i; j++)
                {
                    costGraph[i][j] = costGraph[j][i];
                }
            }

            List<int> pathIndexList = DP(costGraph);

            map.roads = new List<List<Point>>();
            map.kdtree = new KDTree();
            map.road = new List<Point>();
            for(int i = 0; i < pathIndexList.Count; i++)
            {
                int originIndex = pathIndexList[i];
                int goalIndex = pathIndexList[(i + 1) % pathIndexList.Count];
                int temp = originIndex;
                if(originIndex > goalIndex)
                {
                    originIndex = goalIndex;
                    goalIndex = temp;
                }
                int pathIndex = (int)(-originIndex * originIndex / 2.0 + (-0.5 + (pathIndexList.Count - 1)) * originIndex + goalIndex - 1);

                Path path = pathsList[pathIndex];
                
                map.roads.Add(path.road);
            }

            Thread.Sleep(200);
            map.searchFlag = 1;
        }

        public void Search(Path path)
        {
            Stopwatch sw = new Stopwatch();
            KDNode origin = new KDNode(path.origin.data);    // 起点
            KDNode goal = new KDNode(path.goal.data);        // 终点

            // 初始化超椭圆参数
            Hyperellipsoid hyperellipsoid = new Hyperellipsoid(origin, goal);
            // 将起点加入采样图
            //kdtree.Add(origin);
            KDTree kdtree = new KDTree(path.path);
            //map.kdtree = kdtree;

            List<Point> road = new List<Point>();

            Random rnd = new Random();

            double c_best = path.cost;
            for (int iter = 0; iter < ITER; iter++)
            {
                // 保存原始采样点
                KDNode sample0 = GetSample(hyperellipsoid, c_best, rnd);

                // 初始化采样点为原始采样点
                KDNode sample = new KDNode(sample0.x, sample0.y);
                // 最近邻搜索
                //sw.Start();
                sample.front = kdtree.GetNearest(sample.data);
                double distance_near = GetEuclideanDistance(sample.front, sample);
                //sw.Stop();
                if (distance_near > MAX_DISTANCE)
                {
                    // 如果原始采样点与当前节点距离大于最大距离
                    // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                    // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                    double theta = Math.Atan2(sample0.y - sample.front.y, sample0.x - sample.front.x); // 倾角
                    sample.x = sample.front.x + (int)(MAX_DISTANCE * Math.Cos(theta));
                    sample.y = sample.front.y + (int)(MAX_DISTANCE * Math.Sin(theta));
                    sample.data = new List<double> { (double)sample.x, (double)sample.y };
                }

                //if (sample.isEqual(origin))
                //    continue;

                if (!isCollision(sample.front, sample)) // 如果采样点和已知样点图间存在一条无碰撞路径
                {
                    if (sample.isEqual(sample.front))
                        sample = sample.front;
                    else
                    {
                        kdtree.Add(sample);
                        sample.front.neighbor.Add(sample);
                        sample.cost = sample.front.cost + GetEuclideanDistance(sample, sample.front); // 计算代价
                    }
                    
                    sw.Start();
                    // 重连 Rewire
                    double R = OptimizationR;
                    List<KDNode> nearests = kdtree.GetRange(sample.data, R);
                    //List<KDNode> nearests = kdtree.GetKNearest(sample.data, 50);
                    //if (nearests.Count > 50)
                    //    Console.WriteLine("ok!");
                    foreach (KDNode n in nearests) // 遍历所有潜在父节点
                    //foreach (KDNode n in kdtree.kdnodes) // 遍历所有潜在父节点
                    {
                        double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                        if (cost < sample.cost) // 如果新路径代价更小
                        {
                            if (!isCollision(sample, n, R)) // 碰撞检测通过
                            {
                                sample.front.Remove(sample); // 将之前的边删掉
                                sample.cost = cost; // 更新代价
                                sample.front = n; // 更新路径
                                sample.front.neighbor.Add(sample); // 添加新边
                            }
                        }
                        else
                        {
                            cost = sample.cost + GetEuclideanDistance(sample, n); // 计算代价
                            if (cost < n.cost) // 如果新路径代价更小
                            {
                                if (!isCollision(sample, n, R)) // 碰撞检测通过
                                {
                                    //n.front.Remove(n); // 将之前的边删掉
                                    n.cost = cost; // 更新代价
                                    n.front = sample; // 更新路径
                                                      //n.front.neighbor.Add(n); // 添加新边
                                }
                            }
                        }
                    }
                    //foreach (KDNode n in nearests) // 遍历所有潜在父节点
                    //{
                    //    double cost = sample.cost + GetEuclideanDistance(sample, n); // 计算代价
                    //    if (cost < n.cost) // 如果新路径代价更小
                    //    {
                    //        if (!isCollision(sample, n, R)) // 碰撞检测通过
                    //        {
                    //            //n.front.Remove(n); // 将之前的边删掉
                    //            n.cost = cost; // 更新代价
                    //            n.front = sample; // 更新路径
                    //            //n.front.neighbor.Add(n); // 添加新边
                    //        }
                    //    }
                    //}



                    sw.Stop();
                    //Console.WriteLine("Rewire Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
                    //sw.Reset();

                    // 到达终点附近，回溯得到路径
                    if (!isCollision(sample, goal, MAX_DISTANCE))
                    {
                        double c_new = sample.cost + GetEuclideanDistance(goal, sample);

                        if (c_new < c_best) // 新路径的长度更小
                        {
                            path.path = new List<KDNode>();
                            goal.front = sample;
                            // 回溯路径
                            road = new List<Point>();
                            road.Add(goal.Node2Point());
                            path.path.Add(goal);
                            while (sample.front != null)
                            {
                                road.Add(new Point(sample.x, sample.y));
                                sample = sample.front;
                                path.path.Add(sample);
                            }
                            road.Add(sample.Node2Point());
                            path.path.Add(sample);
                            path.road = road;
                            //path.cost = c_new;
                            path.cost = GetCost(goal);

                            map.road = road;

                            c_best = c_new;

                            // 根据路径长度计算迭代次数
                            //ITER = (int)c_best * 5;

                            // 剔除超椭圆外的样点
                            List<KDNode> kdnodes = new List<KDNode>();
                            for (int i = 0; i < kdtree.kdnodes.Count(); i++)
                            {
                                KDNode kdnode = kdtree.kdnodes[i];
                                if (GetEuclideanDistance(kdnode, origin)
                                    + GetEuclideanDistance(kdnode, goal) < c_best)
                                {
                                    kdnodes.Add(kdnode);
                                }
                            }
                            //sw.Start();
                            if (kdnodes.Count != 0)
                                kdtree = new KDTree(kdnodes);
                            map.kdtree = kdtree;
                            //sw.Stop();
                            //Thread.Sleep(20);

                        }
                        Console.WriteLine("iter: " + iter + " c_best: " + c_best + " " + kdtree.kdnodes.Count());
                        //Thread.Sleep(200);
                    }
                }
            }

            Console.WriteLine(kdtree.kdnodes.Count() + " Rewire Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
            //Thread.Sleep(200);
            //map.searchFlag = 1;
            //map.roads.Add(road);
        }
        public double GetCost(KDNode n)
        {
            double cost = 0;
            while (n.front != null)
            {
                cost += GetEuclideanDistance(n, n.front);
                n = n.front;
            }
            return cost;
        }

        List<int> DP(List<List<double>> costGraph)
        {

            List<int> path = new List<int>();
            List<bool> visited = new List<bool>();
            List<List<double>> dp = new List<List<double>>();
            for (int i = 0; i < costGraph.Count; i++)
            {
                visited.Add(false);
                dp.Add(new List<double>());
                for (int j = 0; j < (1 << (costGraph.Count - 1)); j++)
                {
                    dp[i].Add(0);
                }
                dp[i][0] = costGraph[i][0];
            }
            const double INF = 10000;
            for (int j = 1; j < (1 << (costGraph.Count - 1)); j++)
            {
                for (int i = 0; i < costGraph.Count; i++)
                {
                    dp[i][j] = INF;
                    if (((j >> (i - 1)) & 1) == 1)
                    {
                        continue;
                    }

                    for (int k = 1; k < costGraph.Count; k++)
                    {
                        if (((j >> (k - 1)) & 1) == 0)
                        {
                            continue;
                        }
                        if (dp[i][j] > costGraph[i][k] + dp[k][j ^ (1 << (k - 1))])
                        {
                            dp[i][j] = costGraph[i][k] + dp[k][j ^ (1 << (k - 1))];
                        }

                    }
                }
            }

            int pioneer = 0, S = (1 << (costGraph.Count - 1)) - 1, temp = 0;
            double min = INF;
            //把起点结点编号加入容器
            path.Add(0);

            while (!isVisited(visited))
            {
                for (int i = 1; i < costGraph.Count; i++)
                {
                    if (visited[i] == false && (S & (1 << (i - 1))) != 0)
                    {
                        if (min > costGraph[i][pioneer] + dp[i][(S ^ (1 << (i - 1)))])
                        {
                            min = costGraph[i][pioneer] + dp[i][(S ^ (1 << (i - 1)))];
                            temp = i;
                        }
                    }
                }
                pioneer = temp;
                path.Add(pioneer);
                visited[pioneer] = true;
                S = S ^ (1 << (pioneer - 1));
                min = INF;
            }


            return path;
        }
        bool isVisited(List<bool> visited)
        {
            for (int i = 1; i < visited.Count; i++)
            {
                if (visited[i] == false)
                {
                    return false;
                }
            }
            return true;
        }
    }
}

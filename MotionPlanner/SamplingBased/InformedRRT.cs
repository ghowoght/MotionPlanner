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


        int ITER = 10000;                     // 迭代次数 / 采样点数
        const double MAX_DISTANCE = 15;             // 两个节点建立连接的最大距离
        const double OptimizationR = 30;            // 优化半径
        public Graph samplesGraph = new Graph();    // 采样后的样点图

        Hyperellipsoid hyperellipsoid;              // 超椭圆参数

        KDTree kdtree = new KDTree();

        class Hyperellipsoid
        {
            public double c_max = 0;
            public double c_min = 0;

            public double[,] rotationMatrix = new double[2, 2];

            public KDNode centre = new KDNode();

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
        public KDNode RotationToWorldFrame(KDNode sample)
        {
            KDNode sample_new = new KDNode((int)(hyperellipsoid.rotationMatrix[0, 0] * sample.x + hyperellipsoid.rotationMatrix[0, 1] * sample.y + hyperellipsoid.centre.x),
                                        (int)(hyperellipsoid.rotationMatrix[1, 0] * sample.x + hyperellipsoid.rotationMatrix[1, 1] * sample.y + hyperellipsoid.centre.y));
            sample_new.x = sample_new.x < 0 ? 0 : sample_new.x;
            sample_new.x = sample_new.x >= map.Height ? (map.Height - 1) : sample_new.x;
            sample_new.y = sample_new.y < 0 ? 0 : sample_new.y;
            sample_new.y = sample_new.y >= map.Width ? (map.Width - 1) : sample_new.y;

            return sample_new;
        }
        public KDNode GetSample(KDNode origin, KDNode goal, double c_max, Random rnd)
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

                sample = RotationToWorldFrame(sample0);
            } while (map.map[sample.x][sample.y] == (int)GridMap.MapStatus.Occupied);
            return sample;
        }

        public void Search()
        {
            Stopwatch sw = new Stopwatch();
            KDNode origin = new KDNode(map.origin.X, map.origin.Y); // 起点
            KDNode goal = new KDNode(map.goal.X, map.goal.Y);       // 终点

            // 初始化超椭圆参数
            hyperellipsoid = new Hyperellipsoid(origin, goal);
            // 将起点加入采样图
            kdtree.Add(origin);

            Random rnd = new Random();

            double c_best = double.MaxValue;
            for (int iter = 0; iter < ITER; iter++)
            {
                //if (c_best < 263)
                //    break;
                // 保存原始采样点
                KDNode sample0 = GetSample(origin, goal, c_best, rnd); 

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
    }
}

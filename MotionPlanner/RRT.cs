using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MotionPlanner
{
    /// <summary>
    /// 快速扩展随机树算法(Rapidly-exploring Random Tree, RRT)
    /// </summary>
    class RRT
    {

        const int NUM_SAMPLES = 20000; // 采样点数
        const double MAX_DISTANCE = 15; // 两个节点建立连接的最大距离
        public Graph samplesGraph = new Graph();
        GridMap map;
        public RRT(GridMap map)
        {
            this.map = map;
            map.graph = samplesGraph;
        }

        public void Search()
        {
            samplesGraph.nodes.Add(new Node(map.origin.X, map.origin.Y)); // 将起点加入采样图

            Node origin = new Node(map.origin.X, map.origin.Y);
            Node goal = new Node(map.goal.X, map.goal.Y);

            Random rd = new Random();
            while (samplesGraph.nodes.Count < NUM_SAMPLES)
            {
                double p = rd.Next(0, 100) / 100.0;
                int x = 0;
                int y = 0;
                if (p > 0.1)
                {
                    x = rd.Next(0, map.Height);
                    y = rd.Next(0, map.Width);
                }
                else
                {
                    x = goal.x;
                    y = goal.y;
                }
                if (map.map[x][y] != (int)GridMap.MapStatus.Occupied)
                {
                    //samplesGraph.Add(new Node(x, y));
                    //map.graph.Add(new List<Point> { new Point(x, y) });
                    Node sample = new Node(x, y);
                    double distance = GetEuclideanDistance(new Point(samplesGraph.nodes[0].x, samplesGraph.nodes[0].y), new Point(sample.x, sample.y));
                    sample.front = samplesGraph.nodes[0];
                    bool flag = false;
                    for (int i = 0; i < samplesGraph.nodes.Count; i++)
                    {
                        if (!isCollision(new Point(samplesGraph.nodes[i].x, samplesGraph.nodes[i].y), new Point(sample.x, sample.y), MAX_DISTANCE))
                        {
                            flag = true;
                            if (!samplesGraph.nodes[i].isEqual(sample))
                            {
                                double distance1 = GetEuclideanDistance(new Point(samplesGraph.nodes[i].x, samplesGraph.nodes[i].y), new Point(sample.x, sample.y));
                                if (distance1 < distance)
                                {
                                    sample.front = samplesGraph.nodes[i];
                                    distance = distance1;
                                }

                            }
                        }
                    }
                    if (flag)
                    {
                        samplesGraph.nodes.Add(sample);
                        sample.front.neighbor.Add(sample);

                        // 到达终点后，回溯得到路径
                        if (sample.isEqual(goal))
                        {
                            while (sample.front != null)
                            {
                                map.map[sample.x][sample.y] = (int)GridMap.MapStatus.Road;
                                map.road.Add(new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            map.road.Add(map.origin);
                            break;
                        }
                    }




                }
                //Thread.Sleep(1);
                
            }
        }

        private double GetEuclideanDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }

        public bool isCollision(Point p1, Point p2, double maxDist)
        {

            double theta = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X); // 倾角
            double dist = GetEuclideanDistance(p1, p2); // 两点距离
            if (dist > maxDist)
            {
                return true;
            }
            double step = 1; // 步长
            int n_step = (int)(dist / step); // 步数
            PointF p = new PointF((float)p1.X + (float)(step * Math.Cos(theta)),
                                (float)p1.Y + (float)(step * Math.Sin(theta)));
            for (int i = 0; i < n_step; i++)
            {
                if (map.map[(int)Math.Round(p.X)][(int)Math.Round(p.Y)] == (int)GridMap.MapStatus.Occupied)
                {
                    return true;  // 有碰撞
                }

                p.X += (float)(step * Math.Cos(theta));
                p.Y += (float)(step * Math.Sin(theta));
            }
            return false;
        }
    }
}

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
        public Graph samplesGraph = new Graph(); // 采样后的样点图
        GridMap map;
        public RRT(GridMap map)
        {
            this.map = map;
            map.graph = samplesGraph;
        }

        /// <summary>
        /// 搜索路径函数
        /// </summary>
        public void Search()
        {
            //Thread.Sleep(1000);
            Node origin = new Node(map.origin.X, map.origin.Y);
            Node goal = new Node(map.goal.X, map.goal.Y);

            samplesGraph.nodes.Add(origin); // 将起点加入采样图

            Random rd = new Random();
            while (samplesGraph.nodes.Count < NUM_SAMPLES)
            {
                // 采样
                double p = rd.Next(0, 100) / 100.0;
                int x, y;
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
                    Node sample = new Node(x, y);
                    double distance = GetEuclideanDistance(samplesGraph.nodes[0].Node2Point(), sample.Node2Point());
                    sample.front = samplesGraph.nodes[0];

                    Node sample0 = new Node(sample.x, sample.y);
                    sample0.front = samplesGraph.nodes[0];

                    bool flag = false;
                    for (int i = 0; i < samplesGraph.nodes.Count; i++)
                    {
                        
                        Node temp = new Node(sample0.x, sample0.y);

                        double dist = GetEuclideanDistance(samplesGraph.nodes[i].Node2Point(), sample0.Node2Point()); // 两点距离
                        if (dist > MAX_DISTANCE)
                        {
                            double theta = Math.Atan2(sample0.y - samplesGraph.nodes[i].y, sample0.x - samplesGraph.nodes[i].x); // 倾角
                            temp = new Node(samplesGraph.nodes[i].x + (int)((MAX_DISTANCE - 1) * Math.Cos(theta)),
                                                samplesGraph.nodes[i].y + (int)((MAX_DISTANCE - 1) * Math.Sin(theta)));
                            
                        }
                        temp.front = sample.front;

                        if (!isCollision(samplesGraph.nodes[i].Node2Point(), temp.Node2Point(), MAX_DISTANCE))
                        {
                            
                            if (!samplesGraph.nodes[i].isEqual(temp))
                            {
                                flag = true;
                                double distance1 = GetEuclideanDistance(samplesGraph.nodes[i].Node2Point(), sample0.Node2Point());
                                if (distance1 < distance)
                                {
                                    
                                    temp.front = samplesGraph.nodes[i];
                                    sample = temp;
                                    distance = distance1;
                                }
                                if (samplesGraph.nodes.Count == 1)
                                    sample = temp;
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
                Thread.Sleep(1);
                
            }
        }

        /// <summary>
        /// 计算两点间的欧氏距离
        /// </summary>
        /// <param name="p1">点1</param>
        /// <param name="p2">点2</param>
        /// <returns></returns>
        private double GetEuclideanDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }
        /// <summary>
        /// 判断两点间连线是否会经过障碍物，以及两点间距离是否在最大连线范围内
        /// </summary>
        /// <param name="p1">点1</param>
        /// <param name="p2">点2</param>
        /// <param name="maxDist">最大距离</param>
        /// <returns></returns>
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

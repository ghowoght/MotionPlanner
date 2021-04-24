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
    /// RRT-Connect算法
    /// </summary>
    class RRTConnect
    {
        const int ITER = 20000;                 // 迭代次数
        const double MAX_DISTANCE = 10;         // 两个节点建立连接的最大距离
        public Graph samplesGraph = new Graph();// 采样后的样点图
        public Tree originTree = new Tree();    // 以起点作为根节点的树
        public Tree goalTree = new Tree();      // 以终点作为根节点的树
        GridMap map;
        public RRTConnect(GridMap map)
        {
            this.map = map;
            map.graph = samplesGraph;
            map.originTree = originTree;
            map.goalTree = goalTree;
        }

        /// <summary>
        /// 搜索路径函数
        /// </summary>
        public void Search()
        {
            //Thread.Sleep(1000);
            Node origin = new Node(map.origin.X, map.origin.Y); // 起点
            Node goal = new Node(map.goal.X, map.goal.Y); // 终点

            Random rd = new Random();

            originTree.nodes.Add(origin); // 将起点加入采样图
            goalTree.nodes.Add(goal);

            for(int iter = 0; iter < ITER; iter++)
            {
                // 获取原始采样点
                Node sample0 = GetSample(rd);

                if (map.map[sample0.x][sample0.y] != (int)GridMap.MapStatus.Occupied)
                {
                    // 初始化采样点为原始采样点
                    Node sample = new Node(sample0.x, sample0.y);
                    // 将已知样点集的第一个节点作为距离其最近的节点
                    sample.front = originTree.nodes[0];
                    double distance_near = GetEuclideanDistance(originTree.nodes[0], sample);
                    // 寻找最近邻
                    for (int i = 1; i < originTree.nodes.Count; i++)
                    {
                        double distance = GetEuclideanDistance(originTree.nodes[i], sample0);
                        if (distance < distance_near)
                        {
                            sample.front = originTree.nodes[i];
                            distance_near = distance;
                        }

                    }
                    if (distance_near > MAX_DISTANCE)
                    {
                        // 如果原始采样点与当前节点距离大于最大距离
                        // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                        // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                        double theta = Math.Atan2(sample0.y - sample.front.y, sample0.x - sample.front.x); // 倾角
                        sample.x = sample.front.x + (int)(MAX_DISTANCE * Math.Cos(theta));
                        sample.y = sample.front.y + (int)(MAX_DISTANCE * Math.Sin(theta));
                    }
                    if (!isCollision(sample.front, sample)) // 如果采样点和已知样点图间存在一条无碰撞路径
                    {
                        originTree.nodes.Add(sample);
                        sample.front.neighbor.Add(sample);

                        //到达终点后，回溯得到路径
                        if (!isCollision(originTree.nodes.Last(), goalTree.nodes.Last(), MAX_DISTANCE))
                        {

                            sample = originTree.nodes.Last();

                            while (sample.front != null)
                            {
                                map.map[sample.x][sample.y] = (int)GridMap.MapStatus.Road;
                                map.road.Add(new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            map.road.Add(map.origin);

                            sample = goalTree.nodes.Last();

                            while (sample.front != null)
                            {
                                map.map[sample.x][sample.y] = (int)GridMap.MapStatus.Road;
                                map.road.Insert(0, new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            map.road.Insert(0, map.goal);
                            break;
                        }
                    }
                }

                if (map.map[sample0.x][sample0.y] != (int)GridMap.MapStatus.Occupied)
                {
                    // 初始化采样点为原始采样点
                    Node sample = new Node(sample0.x, sample0.y);
                    // 将已知样点集的第一个节点作为距离其最近的节点
                    sample.front = goalTree.nodes[0];
                    double distance_near = GetEuclideanDistance(goalTree.nodes[0], sample);
                    // 寻找最近邻
                    for (int i = 1; i < goalTree.nodes.Count; i++)
                    {
                        double distance = GetEuclideanDistance(goalTree.nodes[i], sample0);
                        if (distance < distance_near)
                        {
                            sample.front = goalTree.nodes[i];
                            distance_near = distance;
                        }

                    }
                    if (distance_near > MAX_DISTANCE)
                    {
                        // 如果原始采样点与当前节点距离大于最大距离
                        // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                        // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                        double theta = Math.Atan2(sample0.y - sample.front.y, sample0.x - sample.front.x); // 倾角
                        sample.x = sample.front.x + (int)(MAX_DISTANCE * Math.Cos(theta));
                        sample.y = sample.front.y + (int)(MAX_DISTANCE * Math.Sin(theta));
                    }
                    if (!isCollision(sample.front, sample)) // 如果采样点和已知样点图间存在一条无碰撞路径
                    {
                        goalTree.nodes.Add(sample);
                        sample.front.neighbor.Add(sample);

                        //到达终点后，回溯得到路径
                        if (!isCollision(originTree.nodes.Last(), goalTree.nodes.Last(), MAX_DISTANCE))
                        {

                            sample = originTree.nodes.Last();

                            while (sample.front != null)
                            {
                                map.map[sample.x][sample.y] = (int)GridMap.MapStatus.Road;
                                map.road.Add(new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            map.road.Add(map.origin);

                            sample = goalTree.nodes.Last();

                            while (sample.front != null)
                            {
                                map.map[sample.x][sample.y] = (int)GridMap.MapStatus.Road;
                                map.road.Insert(0, new Point(sample.x, sample.y));
                                sample = sample.front;
                            }
                            map.road.Insert(0, map.goal);
                            break;
                        }
                    }
                }
                Thread.Sleep(1);

            }
            Thread.Sleep(100);
            map.searchFlag = 1;
        }

        /// <summary>
        /// 随机数种子
        /// </summary>

        /// <summary>
        /// 获取采样点
        /// </summary>
        /// <returns></returns>
        public Node GetSample(Random rd)
        {
            return new Node(rd.Next(0, map.Height), rd.Next(0, map.Width));
        }

        /// <summary>
        /// 计算两个节点间的欧氏距离
        /// </summary>
        /// <param name="n1">节点1</param>
        /// <param name="n2">节点2</param>
        /// <returns></returns>
        private double GetEuclideanDistance(Node n1, Node n2)
        {
            return Math.Sqrt(Math.Pow(n1.x - n2.x, 2) + Math.Pow(n1.y - n2.y, 2));
        }

        /// <summary>
        /// 判断两点间连线是否会经过障碍物，以及两点间距离是否在最大连线范围内
        /// </summary>
        /// <param name="n1">节点1</param>
        /// <param name="n2">节点2</param>
        /// <param name="maxDist">最大距离</param>
        /// <returns></returns>
        public bool isCollision(Node n1, Node n2, double maxDist)
        {

            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x); // 倾角
            double dist = GetEuclideanDistance(n1, n2); // 两点距离
            if (dist > maxDist)
            {
                return true;
            }
            double step = 1; // 步长
            int n_step = (int)(dist / step); // 步数
            PointF p = new PointF((float)n1.x + (float)(step * Math.Cos(theta)),
                                (float)n1.y + (float)(step * Math.Sin(theta)));
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

        /// <summary>
        /// 判断两点间连线是否会经过障碍物
        /// </summary>
        /// <param name="n1">节点1</param>
        /// <param name="n2">节点2</param>
        /// <returns></returns>
        public bool isCollision(Node n1, Node n2)
        {

            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x); // 倾角
            double dist = GetEuclideanDistance(n1, n2); // 两点距离

            double step = 1; // 步长
            int n_step = (int)(dist / step); // 步数
            PointF p = new PointF((float)n1.x + (float)(step * Math.Cos(theta)),
                                (float)n1.y + (float)(step * Math.Sin(theta)));
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

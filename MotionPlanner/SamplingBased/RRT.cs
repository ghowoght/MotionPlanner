﻿using System;
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
        const double MAX_DISTANCE = 10; // 两个节点建立连接的最大距离
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
            Thread.Sleep(1000);
            Node origin = new Node(map.origin.X, map.origin.Y); // 起点
            Node goal = new Node(map.goal.X, map.goal.Y); // 终点

            samplesGraph.nodes.Add(origin); // 将起点加入采样图

            Random rd = new Random();
            while (samplesGraph.nodes.Count < NUM_SAMPLES)
            {
                // 采样
                int x = rd.Next(0, map.Height);
                int y = rd.Next(0, map.Width);

                if (map.map[x][y] != (int)GridMap.MapStatus.Occupied)
                {
                    Node sample0 = new Node(x, y); // 保存原始采样点

                    // 初始化采样点为原始采样点
                    Node sample = new Node(x, y);
                    // 将已知样点集的第一个节点作为距离其最近的节点
                    sample.front = samplesGraph.nodes[0];
                    double distance_near = GetEuclideanDistance(samplesGraph.nodes[0], sample);
                    // 寻找最近邻
                    for (int i = 1; i < samplesGraph.nodes.Count; i++)
                    {
                        double distance = GetEuclideanDistance(samplesGraph.nodes[i], sample0);
                        if (distance < distance_near)
                        {
                            sample.front = samplesGraph.nodes[i];
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
                        samplesGraph.nodes.Add(sample);
                        sample.front.neighbor.Add(sample);

                        // 到达终点后，回溯得到路径
                        //if (sample.isEqual(goal))
                        if (!isCollision(sample, goal, MAX_DISTANCE))
                        {
                            map.road.Add(map.goal);
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
            Thread.Sleep(100);
            map.searchFlag = 1;
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

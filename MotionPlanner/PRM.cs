﻿using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionPlanner
{
    class PRM
    {
        // 概率路图算法

        const int NUM_SAMPLES = 100; // 采样点数
        const double MAX_DISTANCE = 50; // 两个节点建立连接的最大距离
        // 采样后的样点图
        public Graph samplesGraph = new Graph();
        GridMap map;
        public PRM(GridMap map)
        {
            this.map = map;
            map.graph = samplesGraph;
        }

        public void Search()
        {

            samplesGraph.nodes.Add(new Node(map.origin.X, map.origin.Y)); // 将起点加入采样图
            // 采样
            Random rd = new Random();
            while (samplesGraph.nodes.Count < NUM_SAMPLES)
            {
                int x = rd.Next(0, map.Height );
                int y = rd.Next(0, map.Width);
                if (map.map[x][y] != (int)GridMap.MapStatus.Occupied)
                {
                    samplesGraph.nodes.Add(new Node(x, y));
                }
            }

            // 生成概率路图
            Node origin = new Node(map.origin.X, map.origin.Y);
            Node goal = new Node(map.goal.X, map.goal.Y);
            for (int i = 0; i < samplesGraph.nodes.Count; i++)
            {
                foreach(Node node in samplesGraph.nodes)
                {
                    if (!isCollision(new Point(samplesGraph.nodes[i].x, samplesGraph.nodes[i].y), new Point(node.x, node.y), MAX_DISTANCE))
                    {
                        if (!samplesGraph.nodes[i].isEqual(node))
                            samplesGraph.nodes[i].neighbor.Add(node);

                    }

                }
                if (!isCollision(new Point(samplesGraph.nodes[i].x, samplesGraph.nodes[i].y), map.goal, MAX_DISTANCE))
                {
                    samplesGraph.nodes[i].neighbor.Add(goal);
                }
            }

            // 使用Dijkstra算法搜索路径
            PriorityQueue<Node> openList = new PriorityQueue<Node>();
            openList.Push(samplesGraph.nodes[0]);
            while(openList.Count != 0)
            {
                Node node = openList.Pop();
                map.map[node.x][node.y] = (int)GridMap.MapStatus.Explored;
                // 获取邻居结点
                foreach(Node n in node.neighbor)
                {
                    if (map.map[n.x][n.y] == (int)GridMap.MapStatus.Exploring)
                    {
                        for(int i = 0; i < openList.Count; i++)
                        {
                            if(openList[i].isEqual(n))
                            {
                                double cost = node.cost + GetEuclideanDistance(new Point(node.x, node.y), new Point(n.x, n.y));
                                if (openList[i].cost > cost)
                                {
                                    openList[i].cost = cost;
                                    openList[i].front = node;
                                }
                            }
                        }
                    }
                    else if(map.map[n.x][n.y] == (int)GridMap.MapStatus.Unoccupied)
                    {
                        n.front = node;
                        n.cost = node.cost + GetEuclideanDistance(new Point(node.x, node.y), new Point(n.x, n.y));
                        openList.Push(n);

                        map.map[n.x][n.y] = (int)GridMap.MapStatus.Exploring;
                    }
                }

                // 到达终点后，回溯得到路径
                if (node.x == map.goal.X && node.y == map.goal.Y)
                {
                    while (node.front != null)
                    {
                        map.map[node.x][node.y] = (int)GridMap.MapStatus.Road;
                        map.road.Add(new Point(node.x, node.y));
                        node = node.front;
                    }
                    map.road.Add(map.origin);
                    break;
                }

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
            if(dist > maxDist)
            {
                return true;
            }
            double step = 1; // 步长
            int n_step = (int)(dist / step); // 步数
            PointF p = new PointF((float)p1.X + (float)(step * Math.Cos(theta)), 
                                (float)p1.Y + (float)(step * Math.Sin(theta)));
            for (int i = 0; i < n_step; i++)
            {
                if(map.map[(int)Math.Round(p.X)][(int)Math.Round(p.Y)] == (int)GridMap.MapStatus.Occupied)
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
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;
using System.Drawing;

namespace MotionPlanner
{
    class Astar
    {
        public class Node : IComparable<Node>
        {
            public int x = 0;
            public int y = 0;
            public double cost = 0;
            public double G = 0; // 移动代价
            public double H = 0; // 

            public Node front = null;
            public Node(int x, int y)
            {
                this.x = x;
                this.y = y;
            }

            public int CompareTo(Node other)
            {
                return this.cost < other.cost ? -1 : (this.cost == other.cost ? 0 : 1);
            }

            public bool isEqual(Node other) // 判断输入结点和本结点是否相同
            { 
                if(this.x == other.x && this.y == other.y)
                {
                    return true;
                }
                return false;
            }

            
        }

        PriorityQueue<Node> nodes = new PriorityQueue<Node>();
        GridMap map;
        public Astar(GridMap map)
        {
            this.map = map;
        }
        public void Search()
        {
            nodes.Push(new Node(map.origin.X, map.origin.Y));

            while (nodes.Count != 0)
            {
                Node node = nodes.Pop();
                map.map[node.x][node.y] = (int)GridMap.MapStatus.Explored; // 加入CloseList
                GetNeighbors(node); // 扩展周围的结点
                if (node.x == map.goal.X && node.y == map.goal.Y) //判断是否到达目标点
                {
                    while (node.front != null)
                    {
                        map.map[node.x][node.y] = (int)GridMap.MapStatus.Road;
                        map.road.Add(new System.Drawing.Point(node.x, node.y));
                        node = node.front;
                    }
                    map.road.Add(map.origin);
                    break;
                }
                //Thread.Sleep(20);
            }



        }

        private class Motion
        {
            public int delta_x;
            public int delta_y;
            public double delta_cost;
            public Motion(int x, int y, double cost)
            {
                delta_x = x;
                delta_y = y;
                delta_cost = cost;
            }
        }

        Motion[] motionList =
        {
            new Motion(-1,  0,  1),
            new Motion( 0,  1,  1),
            new Motion( 1,  0,  1),
            new Motion( 0, -1,  1),
            new Motion(-1,  1,  Math.Sqrt(2)),
            new Motion( 1,  1,  Math.Sqrt(2)),
            new Motion( 1, -1,  Math.Sqrt(2)),
            new Motion(-1, -1,  Math.Sqrt(2)),
        };

        private int GetManhattanDistance(Point p1, Point p2)
        {
            return Math.Abs(p1.X - p2.X) + Math.Abs(p1.Y - p2.Y);
        }
        private double GetEuclideanDistance(Point p1, Point p2)
        {
            return Math.Sqrt(Math.Pow(p1.X - p2.X, 2) + Math.Pow(p1.Y - p2.Y, 2));
        }
        public void GetNeighbors(Node node)
        {
            foreach (Motion m in motionList) // 遍历周围的邻居结点
            {
                Node n = new Node(node.x + m.delta_x, node.y + m.delta_y);
                n.G = node.G + m.delta_cost;
                n.front = node;
                n.cost = n.G + GetEuclideanDistance(map.goal, new Point(n.x, n.y));

                if (map.map[node.x + m.delta_x][node.y + m.delta_y] == (int)GridMap.MapStatus.Exploring)
                {
                    for(int i = 0; i < nodes.Count; i++) // 遍历队列中的结点，找到该邻居结点在队列中的位置
                    {
                        if(nodes[i].isEqual(n))
                        {
                            if (nodes[i].cost > n.cost)
                                nodes[i] = n;
                        }
                    }
                }
                else if (map.map[node.x + m.delta_x][node.y + m.delta_y] == (int)GridMap.MapStatus.Unoccupied)
                {                    
                    nodes.Push(n);
                    map.map[node.x + m.delta_x][node.y + m.delta_y] = (int)GridMap.MapStatus.Exploring;
                }
            }

        }

        public void Search(Point origin, Point goal)
        {
            nodes.Push(new Node(map.origin.X, map.origin.Y));

            while (nodes.Count != 0)
            {
                Node node = nodes.Pop();
                map.map[node.x][node.y] = (int)GridMap.MapStatus.Explored; // 加入CloseList
                GetNeighbors(node); // 扩展周围的结点
                if (node.x == map.goal.X && node.y == map.goal.Y) //判断是否到达目标点
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
    }
}

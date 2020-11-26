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
                map.map[node.x, node.y] = (int)GridMap.MapStatus.Explored;
                GetNeighbors(node);
                if (node.x == map.goal.X && node.y == map.goal.Y)
                {
                    while (node.front != null)
                    {
                        map.map[node.x, node.y] = (int)GridMap.MapStatus.Road;
                        map.road.Add(new System.Drawing.Point(node.x, node.y));
                        node = node.front;
                    }
                    map.road.Add(map.origin);
                    break;
                }
                Thread.Sleep(20);
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
            foreach (Motion m in motionList)
            {
                if (map.map[node.x + m.delta_x, node.y + m.delta_y] != (int)GridMap.MapStatus.Explored)
                    if (map.map[node.x + m.delta_x, node.y + m.delta_y] != (int)GridMap.MapStatus.Occupied)
                    //if (map.map[node.x + m.delta_x, node.y + m.delta_y] != (int)GridMap.MapStatus.Exploring)
                    {
                        Node n = new Node(node.x + m.delta_x, node.y + m.delta_y);

                        if (n.G == 0)
                        {
                            n.G = node.G + m.delta_cost;
                            n.front = node;
                            n.cost = n.G + GetEuclideanDistance(map.goal, new Point(n.x, n.y));
                            nodes.Push(n);
                            map.map[node.x + m.delta_x, node.y + m.delta_y] = (int)GridMap.MapStatus.Exploring;
                        }
                        else if (n.G > node.G + m.delta_cost)
                        {
                            n.G = node.G + m.delta_cost;
                            n.front = node;
                            n.cost = n.G + GetEuclideanDistance(map.goal, new Point(n.x, n.y));
                        }
                    }
            }

        }
    }
}

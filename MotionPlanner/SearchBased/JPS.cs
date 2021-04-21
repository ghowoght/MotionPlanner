using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MotionPlanner
{
    class JPS
    {
        public class Node : IComparable<Node>
        {
            public int x = 0;
            public int y = 0;
            public double cost = 0;
            public double G = 0; // 移动代价
            public double H = 0; // 

            public List<Motion> forceList = new List<Motion>();

            public Node front = null;
            public Node(int x, int y)
            {
                this.x = x;
                this.y = y;
            }
            public Node(int x, int y, double cost)
            {
                this.x = x;
                this.y = y;
                this.cost = cost;
            }

            public Node Clone()
            {
                Node node = new Node(x, y, cost);
                node.G = G;
                node.forceList = forceList;
                node.front = front;
                node.H = H;

                return node;
            }

            public int CompareTo(Node other)
            {
                return this.cost < other.cost ? -1 : (this.cost == other.cost ? 0 : 1);
            }

            public bool isEqual(Node other) // 判断输入结点和本结点是否相同
            {
                if (this.x == other.x && this.y == other.y)
                {
                    return true;
                }
                return false;
            }


        }

        PriorityQueue<Node> openlist = new PriorityQueue<Node>();
        GridMap map;
        public JPS(GridMap map)
        {
            this.map = map;
        }
        public void Search()
        {
            Node goal = new Node(map.goal.X, map.goal.Y);
            openlist.Push(new Node(map.origin.X, map.origin.Y));
            openlist[0].forceList.Add(motionList[0]);
            openlist[0].forceList.Add(motionList[1]);
            openlist[0].forceList.Add(motionList[2]);
            openlist[0].forceList.Add(motionList[3]);
            bool flag = false;

            while (openlist.Count != 0)
            {
                if (flag) break;
                Node node = openlist.Pop();

                
                foreach (Motion m in node.forceList)
                {
                    if (flag) break;
                    Node n = new Node(node.x, node.y, node.G + GetManhattanDistance(node, goal));
                    n.G = node.G;
                    n.front = node;
                    while (Getopenlisttatus(n) != (int)GridMap.MapStatus.Occupied)
                    {
                        //if (n.x == 11 && n.y == 15)
                        //    Console.WriteLine(node.forceList.Count());
                        if (flag) break;
                        // 垂直跳跃
                        Node current = new Node(n.x, n.y, n.G + GetManhattanDistance(n, goal));
                        current.G = n.G;
                        current.front = n;
                        while (Getopenlisttatus(current) == (int)GridMap.MapStatus.Unoccupied
                            || Getopenlisttatus(current) == (int)GridMap.MapStatus.Exploring
                            || current.isEqual(n) && Getopenlisttatus(current) == (int)GridMap.MapStatus.Explored)
                        {

                            Motion motion = GetForcedNeighbor(current, m, 0);
                            if (motion != null)
                            {
                                current.forceList.Add(motion);
                                if (n.x != node.x && n.y != node.y)
                                {
                                    map.map[n.x][n.y] = (int)GridMap.MapStatus.Exploring;
                                    //openlist.Push(n);
                                }
                                map.map[current.x][current.y] = (int)GridMap.MapStatus.Exploring;
                                openlist.Push(current);
                                break;
                            }
                            if (Getopenlisttatus(current) == (int)GridMap.MapStatus.Unoccupied)
                                map.map[current.x][current.y] = (int)GridMap.MapStatus.Explored;

                            current = current.Clone();
                            current.x += m.delta_x;
                            current.G += 1;
                            current.cost = current.G + GetManhattanDistance(current, goal);

                            if (current.x == map.goal.X && current.y == map.goal.Y)
                            {
                                while (current.front != null)
                                {
                                    //map.map[current.x][current.y] = (int)GridMap.MapStatus.Road;
                                    map.road.Add(new Point(current.x, current.y));
                                    current = current.front;
                                }
                                map.road.Add(map.origin);
                                flag = true;
                                break;
                            }
                            //Thread.Sleep(20);
                        }
                        if (flag) break;
                        // 水平跳跃
                        current = new Node(n.x, n.y, n.G + GetManhattanDistance(n, goal));
                        current.G = n.G;
                        current.front = n;
                        while (Getopenlisttatus(current) == (int)GridMap.MapStatus.Unoccupied
                            || Getopenlisttatus(current) == (int)GridMap.MapStatus.Exploring
                            || current.isEqual(n) && Getopenlisttatus(current) == (int)GridMap.MapStatus.Explored)
                        {
                            Motion motion = GetForcedNeighbor(current, m, 1);
                            if (motion != null)
                            {
                                current.forceList.Add(motion);
                                if (n.x != node.x && n.y != node.y)
                                {
                                    map.map[n.x][n.y] = (int)GridMap.MapStatus.Exploring;
                                    //openlist.Push(n);
                                }
                                map.map[current.x][current.y] = (int)GridMap.MapStatus.Exploring;
                                openlist.Push(current);
                                break;
                            }
                            if (Getopenlisttatus(current) == (int)GridMap.MapStatus.Unoccupied)
                                map.map[current.x][current.y] = (int)GridMap.MapStatus.Explored;

                            current = current.Clone();
                            current.y += m.delta_y;
                            current.G += 1;
                            current.cost = current.G + GetManhattanDistance(current, goal);

                            if (current.x == map.goal.X && current.y == map.goal.Y)
                            {
                                while (current.front != null)
                                {
                                    //map.map[current.x][current.y] = (int)GridMap.MapStatus.Road;
                                    map.road.Add(new Point(current.x, current.y));
                                    current = current.front;
                                }
                                map.road.Add(map.origin);
                                flag = true;
                                break;
                            }
                            //Thread.Sleep(20);
                        }

                        n = n.Clone();
                        n.x += m.delta_x;
                        n.y += m.delta_y;
                        n.G += Math.Sqrt(2);
                        n.cost = n.G + GetManhattanDistance(n, goal);
                    }
                }
            }
        }

        private int GetManhattanDistance(Node p1, Node p2)
        {
            return Math.Abs(p1.x - p2.x) + Math.Abs(p1.y - p2.y);
        }

        private bool hasForcedNeighbor(Node current, Motion m, int jumpFlag)
        {

            if(jumpFlag == 0) // 垂直跳跃
            {
                if (map.map[current.x][current.y + 1] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x + m.delta_x][current.y + 1] != (int)GridMap.MapStatus.Occupied
                    || map.map[current.x][current.y - 1] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x + m.delta_x][current.y - 1] != (int)GridMap.MapStatus.Occupied)
                {
                    return true;
                }
            }
            else if(jumpFlag == 1) // 水平跳跃
            {
                if (map.map[current.x + 1][current.y] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x + 1][current.y + m.delta_y] != (int)GridMap.MapStatus.Occupied
                    || map.map[current.x - 1][current.y] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x - 1][current.y + m.delta_y] != (int)GridMap.MapStatus.Occupied)
                {
                    return true;
                }
            }
            return false;
        }
        private Motion GetForcedNeighbor(Node current, Motion m, int jumpFlag)
        {

            if (jumpFlag == 0) // 垂直跳跃
            {
                if (map.map[current.x][current.y + 1] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x + m.delta_x][current.y + 1] == (int)GridMap.MapStatus.Unoccupied)
                {
                    return new Motion(m.delta_x, 1, Math.Sqrt(2));
                }
                else if (map.map[current.x][current.y - 1] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x + m.delta_x][current.y - 1] == (int)GridMap.MapStatus.Unoccupied)
                {
                    return new Motion(m.delta_x, -1, Math.Sqrt(2));
                }
                
            }
            else if (jumpFlag == 1) // 水平跳跃
            {
                if (map.map[current.x + 1][current.y] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x + 1][current.y + m.delta_y] == (int)GridMap.MapStatus.Unoccupied)
                {
                    return new Motion(1, m.delta_y, Math.Sqrt(2));
                }
                else if (map.map[current.x - 1][current.y] == (int)GridMap.MapStatus.Occupied
                    && map.map[current.x - 1][current.y + m.delta_y] == (int)GridMap.MapStatus.Unoccupied)
                {
                    return new Motion(-1, m.delta_y, Math.Sqrt(2));

                }
            }
            return null; 
        }
        int Getopenlisttatus(Node n)
        {
            return map.map[n.x][n.y];
        }

        public class Motion
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

        List<Motion> motionList = new List<Motion>
        {
            new Motion(-1,  1,  Math.Sqrt(2)),
            new Motion( 1,  1,  Math.Sqrt(2)),
            new Motion( 1, -1,  Math.Sqrt(2)),
            new Motion(-1, -1,  Math.Sqrt(2)),
        };

    
    }
}

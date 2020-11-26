using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Threading;

namespace MotionPlanner
{
    class Dijkstra
    {
        public class Node: IComparable<Node>
        {
            public int x = 0;
            public int y = 0;
            public double cost = 0;
            public Node front = null;
            public Node(int x, int y)
            {
                this.x = x;
                this.y = y;
            }

            public int Compare(Node x, Node y)
            {
                return x.cost < y.cost ? -1 : (x.cost == y.cost ? 0 : 1);
            }

            public int CompareTo(Node other)
            {
                return this.cost < other.cost ? -1 : (this.cost == other.cost ? 0 : 1);
            }
        }

        PriorityQueue<Node> nodes = new PriorityQueue<Node>();
        GridMap map;
        public Dijkstra(GridMap map)
        {
            this.map = map;
        }
        public void Search()
        {
            nodes.Push(new Node(map.origin.X, map.origin.Y));

            while(nodes.Count != 0)
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

        public void GetNeighbors(Node node)
        {
            foreach(Motion m in motionList)
            {
                if(map.map[node.x + m.delta_x, node.y + m.delta_y] == 0)
                {
                    Node n = new Node(node.x + m.delta_x, node.y + m.delta_y);
                    n.front = node;
                    n.cost = node.cost + m.delta_cost;
                    nodes.Push(n);
                    map.map[node.x + m.delta_x, node.y + m.delta_y] = (int)GridMap.MapStatus.Exploring;
                }
            }

        }

    }
}

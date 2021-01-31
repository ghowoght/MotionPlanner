using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MotionPlanner
{

    class BFS
    {
        public class Node
        {
            public int x;
            public int y;
            public Node front = null;
            public Node(int x, int y)
            {
                this.x = x;
                this.y = y;
            }
        }
        GridMap map;
        Queue<Node> nodes = new Queue<Node>();

        public BFS(GridMap map)
        {
            this.map = map;
        }
        public void Search()
        {
            nodes.Enqueue(new Node(map.origin.X, map.origin.Y));

            while (nodes.Count != 0)
            {
                Node node = nodes.Dequeue();
                map.map[node.x][node.y] = (int)GridMap.MapStatus.Explored;
                GetNeighbors(node);
                if (node.x == map.goal.X && node.y == map.goal.Y)
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
                System.Threading.Thread.Sleep(20);
            }
            Thread.Sleep(100);
            map.searchFlag = 1;
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
        };

        public void GetNeighbors(Node node)
        {
            foreach (Motion m in motionList)
            {
                if (map.map[node.x + m.delta_x][node.y + m.delta_y] == 0)
                {
                    Node n = new Node(node.x + m.delta_x, node.y + m.delta_y);
                    n.front = node;
                    nodes.Enqueue(n);
                    map.map[node.x + m.delta_x][node.y + m.delta_y] = (int)GridMap.MapStatus.Exploring;
                }
            }

        }

    }
}

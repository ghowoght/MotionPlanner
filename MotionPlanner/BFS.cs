using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionPlanner
{
    class Node
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
    class BFS
    {
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
                GetNeighbors(node);
                if(node.x == map.goal.X && node.y == map.goal.Y)
                {
                    while(node.front != null)
                    {
                        map.map[node.x, node.y] = (int)GridMap.MapStatus.Road;
                        node = node.front;
                    }
                    break;
                }
            }
        }
        public void GetNeighbors(Node node)
        {
            if (map.map[node.x - 1, node.y] == 0) // 上
            {
                Node n = new Node(node.x - 1, node.y);
                n.front = node;
                nodes.Enqueue(n);
                map.map[node.x - 1, node.y] = (int)GridMap.MapStatus.Exploring;
            }
            if (map.map[node.x, node.y + 1] == 0) // 右
            {
                Node n = new Node(node.x, node.y + 1);
                n.front = node;
                nodes.Enqueue(n);
                map.map[node.x, node.y + 1] = (int)GridMap.MapStatus.Exploring;
            }
            if (map.map[node.x + 1, node.y] == 0) // 下
            {
                Node n = new Node(node.x + 1, node.y);
                n.front = node;
                nodes.Enqueue(n);
                map.map[node.x + 1, node.y] = (int)GridMap.MapStatus.Exploring;
            }
            if (map.map[node.x, node.y - 1] == 0) // 左
            {
                Node n = new Node(node.x, node.y - 1);
                n.front = node;
                nodes.Enqueue(n);
                map.map[node.x, node.y - 1] = (int)GridMap.MapStatus.Exploring;
            }
            System.Threading.Thread.Sleep(10);
        }
    }
}

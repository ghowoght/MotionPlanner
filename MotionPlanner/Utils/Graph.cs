using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionPlanner
{
    public class Node : IComparable<Node>
    {
        public int x = 0;
        public int y = 0;
        public double cost = 0;
        public List<Node> neighbor = new List<Node>();
        public Node front = null;
        public Node() { }
        public Node(int x, int y)
        {
            this.x = x;
            this.y = y;
        }

        public void Remove(Node node)
        {
            for (int i = 0; i < neighbor.Count; i++)
            {
                if (neighbor[i].x == node.x && neighbor[i].y == node.y)
                {
                    neighbor.RemoveAt(i);
                }
            }
        }

        /// <summary>
        /// 将Node转换为Point形式
        /// </summary>
        /// <returns>Point</returns>
        public Point Node2Point()
        {
            return new Point(x, y);
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
    public class Graph
    {
        public List<Node> nodes = new List<Node>();
    }
}

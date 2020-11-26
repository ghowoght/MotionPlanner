using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionPlanner
{
    class GridMap
    {
        public int[,] map;
        public Point origin; // 起点
        public Point goal; // 目标点

        public enum MapStatus { Unoccupied = 0, Occupied, Explored}; // 未被占据的、被占据的、已探索的
        
        public GridMap()
        {
            const int N = 20;
            const int M = 40;
            map = new int[N, M];
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    map[i, j] = 0;
                }
            }
            map[10, 10] = 1;
            origin = new Point(2, 10);
            goal = new Point(15, 35);
        }

        public GridMap(int[,] map, Point origin, Point goal)
        {
            this.map = map;
            this.origin = origin;
            this.goal = goal;
        }

        public GridMap(string filename)
        {
            
        }
    }
}

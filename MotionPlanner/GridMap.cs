using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
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
        const int N = 20;
        const int M = 40;

        public enum MapStatus { Unoccupied = 0, Occupied, Explored}; // 未被占据的、被占据的、已探索的
        
        public GridMap()
        {
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
            map = new int[N, M];
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    map[i, j] = 0;
                }
            }
            origin = new Point(2, 10);
            goal = new Point(15, 35);

            // 读取本地地图数据
            StreamReader sr = new StreamReader(filename, Encoding.Default);
            String line;
            while ((line = sr.ReadLine()) != null)
            {
                string[] str = line.ToString().Split(' ');
                map[Convert.ToInt16(str[1]), Convert.ToInt16(str[2])] = Convert.ToInt16(str[0]);
            }
            sr.Close();
        }

        public void SaveMap(string filename)
        {
            FileStream fs = new FileStream("../../map.txt", FileMode.Create);
            StreamWriter sw = new StreamWriter(fs);
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    if (map[i, j] != 0)
                    {
                        sw.WriteLine(map[i, j] + " " + i + " " + j);
                    }
                }
            } 
            //清空缓冲区
            sw.Flush();
            //关闭流
            sw.Close();
            fs.Close();
        }
    }
}

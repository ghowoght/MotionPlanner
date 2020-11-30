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
        //public int[,] map;
        public List<List<int>> map;
        int Height = 20;
        int Width = 40;
        public Point origin = new Point(5, 10);    // 起点
        public Point goal = new Point(15, 35);     // 目标点
        string config_dir = "../../map.txt";

        public List<Point> road = new List<Point>();
        
        public enum MapStatus { Unoccupied = 0, // 未被占据的结点
                                Occupied,       // 被占据的结点
                                Exploring,      // 在容器中的结点
                                Explored,       // 已扩展的结点(已经从容器中弹出的结点)
                                Road            // 路径上的结点
                                }; 
        
        //public GridMap()
        //{
        //    map = new int[N, M];
        //    for (int i = 0; i < N; i++)
        //    {
        //        for (int j = 0; j < M; j++)
        //        {
        //            map[i, j] = 0;
        //        }
        //    }
        //    map[10, 10] = 1;
        //}

        public GridMap(int height, int width, string config_dir)
        {
            this.config_dir = config_dir;
            Height = height;
            Width = width;
            map = new List<List<int>>(Height);
            for(int i = 0; i < Height; i++)
            {
                List<int> temp = new List<int>();
                for (int j = 0; j < Width; j++)
                    temp.Add(0);
                map.Add(temp);
            }
            // 读取本地地图数据
            StreamReader sr = new StreamReader(config_dir, Encoding.Default);
            String line;
            while ((line = sr.ReadLine()) != null)
            {
                string[] str = line.ToString().Split(' ');
                map[Convert.ToInt16(str[1])][Convert.ToInt16(str[2])] = Convert.ToInt16(str[0]);
            }
            sr.Close();

        }

        //public GridMap(int[,] map, Point origin, Point goal)
        //{
        //    this.map = map;
        //    this.origin = origin;
        //    this.goal = goal;
        //}

        //public GridMap(string filename) // 从本地配置中初始化
        //{
        //    map = new int[N, M];
        //    for (int i = 0; i < N; i++)
        //    {
        //        for (int j = 0; j < M; j++)
        //        {
        //            map[i, j] = 0;
        //        }
        //    }

        //    // 读取本地地图数据
        //    StreamReader sr = new StreamReader(filename, Encoding.Default);
        //    String line;
        //    while ((line = sr.ReadLine()) != null)
        //    {
        //        string[] str = line.ToString().Split(' ');
        //        map[Convert.ToInt16(str[1]), Convert.ToInt16(str[2])] = Convert.ToInt16(str[0]);
        //    }
        //    sr.Close();
        //}

        public void SaveMap()
        {
            FileStream fs = new FileStream(config_dir, FileMode.Create);
            StreamWriter sw = new StreamWriter(fs);
            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    if (map[i][j] == 1) // 只保存被占据状态的栅格
                    {
                        sw.WriteLine(map[i][j] + " " + i + " " + j);
                    }
                }
            } 
            //清空缓冲区
            sw.Flush();
            //关闭流
            sw.Close();
            fs.Close();
        }
        public void Reset(string config_dir) // 重置地图
        {
            this.config_dir = config_dir;
            //map = new int[N][M];
            for (int i = 0; i < Height; i++)
            {
                for (int j = 0; j < Width; j++)
                {
                    map[i][j] = 0;
                }
            }

            // 读取地图数据
            StreamReader sr = new StreamReader(config_dir, Encoding.Default);
            String line;
            while ((line = sr.ReadLine()) != null)
            {
                string[] str = line.ToString().Split(' ');
                map[Convert.ToInt16(str[1])][Convert.ToInt16(str[2])] = Convert.ToInt16(str[0]);
            }
            sr.Close();

            road = new List<Point>(); // 清除路径
        }
    }
}

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MotionPlanner
{
    public class GridMap
    {
        /// <summary>
        ///     |
        /// ----|----+y
        ///     |
        ///     +x
        /// </summary>


        //public int[,] map;
        public List<List<int>> map;
        public int Height = 20;
        public int Width = 40;
        public Point origin = new Point(5, 10);    // 起点
        public Point goal = new Point(195, 350);     // 目标点 100, 200
        string config_dir = "../../map.txt";

        public List<Point> road = new List<Point>(); // 路径
        public Graph graph = new Graph(); // 图
        public Tree originTree = new Tree();
        public Tree goalTree = new Tree();

        public KDTree kdtree = new KDTree();

        public int searchFlag = -1; // 搜索标志

        public enum MapStatus { Unoccupied = 0, // 未被占据的结点
                                Occupied,       // 被占据的结点
                                Exploring,      // 在容器中的结点
                                Explored,       // 已扩展的结点(已经从容器中弹出的结点)
                                Road            // 路径上的结点
                                }; 

        public GridMap(string config_dir)
        {
            this.config_dir = config_dir;

            // 读取本地地图数据
            StreamReader sr = new StreamReader(config_dir, Encoding.Default);
            String line;

            // 尺寸
            line = sr.ReadLine();
            string[] str = line.ToString().Split(' ');
            Height = Convert.ToInt16(str[0]);
            Width = Convert.ToInt16(str[1]);

            map = new List<List<int>>(Height);
            for (int i = 0; i < Height; i++)
            {
                List<int> temp = new List<int>();
                for (int j = 0; j < Width; j++)
                    temp.Add(0);
                map.Add(temp);
            }

            // 起点
            line = sr.ReadLine();
            str = line.ToString().Split(' ');
            origin.X = Convert.ToInt16(str[0]);
            origin.Y = Convert.ToInt16(str[1]);
            // 终点
            line = sr.ReadLine();
            str = line.ToString().Split(' ');
            goal.X = Convert.ToInt16(str[0]);
            goal.Y = Convert.ToInt16(str[1]);
            while ((line = sr.ReadLine()) != null)
            {
                str = line.ToString().Split(' ');
                map[Convert.ToInt16(str[1])][Convert.ToInt16(str[2])] = Convert.ToInt16(str[0]);
            }
            sr.Close();

        }

        public void SaveMap()
        {
            FileStream fs = new FileStream(config_dir, FileMode.Create);
            StreamWriter sw = new StreamWriter(fs);
            sw.WriteLine(Height + " " + Width); // 尺寸
            sw.WriteLine(origin.X + " " + origin.Y); // 起点
            sw.WriteLine(goal.X + " " + goal.Y); // 终点
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

            // 尺寸
            line = sr.ReadLine();
            string[] str = line.ToString().Split(' ');
            Height = Convert.ToInt16(str[0]);
            Width = Convert.ToInt16(str[1]);
            // 起点
            line = sr.ReadLine();
            str = line.ToString().Split(' ');
            origin.X = Convert.ToInt16(str[0]);
            origin.Y = Convert.ToInt16(str[1]);
            // 终点
            line = sr.ReadLine();
            str = line.ToString().Split(' ');
            goal.X = Convert.ToInt16(str[0]);
            goal.Y = Convert.ToInt16(str[1]);

            while ((line = sr.ReadLine()) != null)
            {
                str = line.ToString().Split(' ');
                map[Convert.ToInt16(str[1])][Convert.ToInt16(str[2])] = Convert.ToInt16(str[0]);
            }
            sr.Close();

            road = new List<Point>(); // 清除路径
        }
    }
}

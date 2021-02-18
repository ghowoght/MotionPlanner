using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;

using AnimatedGif;
using System.Diagnostics;

namespace MotionPlanner
{
    public partial class MotionPlanner : Form
    {
        public MotionPlanner()
        {
            InitializeComponent();
        }

        private void MotionPlanner_Load(object sender, EventArgs e)
        {
            List<List<double>> dataArray = new List<List<double>>();
            KDTree tree = new KDTree();

            Random rnd = new Random();

            const int PointNum = 30000;

            Stopwatch sw0 = new Stopwatch();
            sw0.Start();

            for (int i = 0; i < PointNum; i++)
            {
                dataArray.Add(new List<double>());
                dataArray[i].Add(rnd.NextDouble() * 100);
                dataArray[i].Add(rnd.NextDouble() * 100);
                dataArray[i].Add(rnd.NextDouble() * 100);
                //tree.Add(dataArray[i]);
            }
            
            tree = new KDTree(dataArray);
            sw0.Stop();
            Console.WriteLine("KD-Tree构建耗时  : " + sw0.Elapsed.TotalMilliseconds + "ms");

            for (int k = 0; k < 1; k++)
            {
                List<double> point = new List<double> { rnd.NextDouble() * 100,
                                                        rnd.NextDouble() * 100,
                                                        rnd.NextDouble() * 100};

                Stopwatch sw = new Stopwatch();
                sw.Start();
                //tree.Add(new List<double>{  rnd.NextDouble() * 100,
                //                            rnd.NextDouble() * 100,
                //                            rnd.NextDouble() * 100});
                KDNode node = tree.GetNearest(point);
                sw.Stop();

                Console.WriteLine("KD-Tree最近邻查找");
                Console.WriteLine("查找点: " + new KDNode(point).ToString());
                Console.WriteLine("最近邻: " + node.ToString());
                Console.WriteLine("距离  : " + KDTree.GetDistance(point, node.data));
                Console.WriteLine("耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");

                sw.Reset();
                sw.Start();
                KDNode nearest = new KDNode();
                double minDist = KDTree.GetDistance(point, dataArray[0]);
                for (int i = 1; i < dataArray.Count; i++)
                {
                    if (minDist > KDTree.GetDistance(point, dataArray[i]))
                    {
                        nearest.data = dataArray[i];
                        minDist = KDTree.GetDistance(point, dataArray[i]);
                    }
                }
                sw.Stop();
                Console.WriteLine("暴力搜索");
                Console.WriteLine("查找点: " + new KDNode(point).ToString());
                Console.WriteLine("最近邻: " + nearest.ToString());
                Console.WriteLine("距离  : " + KDTree.GetDistance(point, nearest.data));
                Console.WriteLine("耗时  : " + sw.Elapsed.TotalMilliseconds + "ms");
                Console.WriteLine("-------------------");

            }

            // K个最近邻搜索


            // 搜索距离输入节点指定范围内的所有节点
            Console.WriteLine("搜索距离输入节点指定范围内的所有节点");
            List<double> point2 = new List<double> {    rnd.NextDouble() * 100,
                                                        rnd.NextDouble() * 100,
                                                        rnd.NextDouble() * 100};
            Console.WriteLine("查找点: " + new KDNode(point2).ToString());
            double R = 10;
            sw0.Reset();
            sw0.Start();
            List<KDNode> nears = tree.GetRange(point2, R);
            sw0.Stop();
            Console.WriteLine("KD-Tree搜索");
            Console.WriteLine("耗时  : " + sw0.Elapsed.TotalMilliseconds + "ms");
            Console.WriteLine("找到  : " + nears.Count + "个");
            
            //foreach (KDNode node in nears)
            //{
            //    Console.WriteLine("最近邻: " + node.ToString() + " " + KDTree.GetDistance(point2, node.data));
            //}

            List<List<double>> nears2 = new List<List<double>>();
            
            sw0.Reset();
            sw0.Start();
            for (int i = 1; i < dataArray.Count; i++)
            {
                if (R > KDTree.GetDistance(point2, dataArray[i]))
                {
                    nears2.Add(dataArray[i]);
                }
            }
            sw0.Stop();
            Console.WriteLine("暴力搜索");
            Console.WriteLine("耗时  : " + sw0.Elapsed.TotalMilliseconds + "ms");
            Console.WriteLine("找到  : " + nears2.Count + "个");
            //foreach (List<double> node in nears2)
            //{
            //    Console.WriteLine(KDTree.GetDistance(point2, node));
            //}


            return;
            //KDTree.preOrderTraveral(tree.root);


            //this.pcb_display.Size = new Size(800, 400);
            //this.Size = new Size(835, 450);


            //GridMap gridMap = new GridMap("../../map/graph/blank_map_2.txt");
            //GraphPainter painter = new GraphPainter(pcb_display, gridMap);

            //GridMap gridMap = new GridMap("../../map/sampling/map4.txt");
            //SamplingPainter painter = new SamplingPainter(pcb_display, gridMap);

            //gridMap.Reset("../../map/sampling/map.txt");


            //gridMap.Reset("../../blank_map.txt");

            //BFS bfs = new BFS(gridMap);
            //new Thread(bfs.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //GBFS gbfs = new GBFS(gridMap);
            //new Thread(gbfs.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //DFS dfs = new DFS(gridMap);
            //new Thread(dfs.Search)
            //{
            //    IsBackground = true
            //}.Start();


            //Dijkstra dijkstra = new Dijkstra(gridMap);
            //new Thread(dijkstra.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //Astar astar = new Astar(gridMap);
            //new Thread(astar.Search)
            //{
            //    IsBackground = true
            //}.Start();


            //JPS jps = new JPS(gridMap);
            //new Thread(jps.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //PRM prm = new PRM(gridMap);
            //new Thread(prm.Search)
            //{
            //    IsBackground = true
            //}.Start();


            //RRT rrt = new RRT(gridMap);
            //new Thread(rrt.Search)
            //{
            //    IsBackground = true
            //}.Start();


            //RRTstar rrtstar = new RRTstar(gridMap);
            //new Thread(rrtstar.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //RRTConnect rrtstar = new RRTConnect(gridMap);
            //new Thread(rrtstar.Search)
            //{
            //    IsBackground = true
            //}.Start();

            //RRTConnectStar rrtstar = new RRTConnectStar(gridMap);
            //new Thread(rrtstar.Search)
            //{
            //    IsBackground = true
            //}.Start();

        }

        private void btn_saveImg_Click(object sender, EventArgs e)
        {
            String time = DateTime.Now.ToString("yyyy-MM-dd_hh-mm-ss");
            pcb_display.Image.Save("img/img_" + time + ".png", System.Drawing.Imaging.ImageFormat.Png);

        }
    }
}

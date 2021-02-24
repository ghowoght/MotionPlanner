﻿using System;
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


            //this.pcb_display.Size = new Size(800, 400);
            //this.Size = new Size(835, 450);


            //GridMap gridMap = new GridMap("../../map/graph/blank_map_2.txt");
            //GraphPainter painter = new GraphPainter(pcb_display, gridMap);

            GridMap gridMap = new GridMap("../../map/sampling/map6.txt");
            SamplingPainter painter = new SamplingPainter(pcb_display, gridMap);

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

            InformedRRT rrt = new InformedRRT(gridMap);
            //new Thread(rrt.Search)
            //{
            //    IsBackground = true
            //}.Start();

            List<KDNode>  goals = new List<KDNode> {new KDNode(182, 73),
                                                    new KDNode(32, 158),
                                                    new KDNode(33, 327),
                                                    new KDNode(116, 272),
                                                    new KDNode(81, 239),
                                                    new KDNode(183, 308),
                                                    new KDNode(98, 153),
                                                    new KDNode(173, 386),
                                                    new KDNode(85, 222),
                                                    new KDNode(6, 272),
                                                    new KDNode(67, 204),
                                                    new KDNode(71, 141),
                                                    new KDNode(143, 20),
                                                    new KDNode(17, 20),
                                                    new KDNode(190, 115),
                                                    //new KDNode(75, 289),
                                                    //new KDNode(128, 118),
                                                    //new KDNode(46, 99),
                                                    //new KDNode(175, 243),
                                                    //new KDNode(95, 357),
                                                    new KDNode(152, 184)};
            Stopwatch sw = new Stopwatch();
            sw.Start();
            rrt.Search(goals);
            //rrt.Search(new List<KDNode>());
            sw.Stop();
            Console.WriteLine("Total Consuming: " + sw.Elapsed.TotalMilliseconds + "ms");
            //SamplingPainter painter = new SamplingPainter(pcb_display, gridMap);
            //KDTree.Test();

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
            String time = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
            pcb_display.Image.Save("img/img_" + time + ".png", System.Drawing.Imaging.ImageFormat.Png);

        }
    }
}

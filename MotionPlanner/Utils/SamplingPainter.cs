using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;
using System.Drawing;
using AnimatedGif;

namespace MotionPlanner
{
    /// <summary>
    /// 针对采样算法的画板类
    /// </summary>
    public class SamplingPainter : BasePainter
    {
        public SamplingPainter(PictureBox pcb_, GridMap map_) // 初始化Painter 
        {
            N = map_.Height;
            M = map_.Width;
            pcb = pcb_;
            gridMap = map_;
            int height = pcb.Height;
            int width = pcb.Width;

            // 初始化地图
            pcb.Image = new Bitmap(width, height);
            Graphics g = Graphics.FromImage(pcb.Image);
            g = PaintMap(g);

            pcb.MouseClick += new MouseEventHandler(pcb_MouseClick);

            new Thread(PainterRefresh).Start();
        }

        public override void PainterRefresh()
        {
            Bitmap image = new Bitmap(pcb.Image);
            Graphics g = Graphics.FromImage(image);

            // 生成GIF
            String time = DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss");
            AnimatedGifCreator agc = new AnimatedGifCreator("img/img_" + time + ".gif");
            agc.AddFrameAsync((Image)image.Clone(), 1000);

            while (true)
            {
                try
                {
                    // 画出地图
                    g = PaintMap(g);
                    // 画出图结构
                    g = PaintGraph(g);
                    // 画出KDTree
                    g = PaintKDTree(g);
                    // 画出树结构
                    g = PaintTree(g);
                    // 画出路径
                    g = PaintRoad(g);

                    pcb.Image = image;

                    Thread.Sleep(50);

                    if (gridMap.searchFlag != -1)
                    {
                        agc.AddFrameAsync((Image)image.Clone(), 1200);
                        agc.Dispose();
                        break;
                    }
                    else
                    {
                        agc.AddFrameAsync((Image)image.Clone(), 50);
                    }
                }
                catch { }





            }

        }

        public override Graphics PaintMap(Graphics g)
        {
            //Graphics g = Graphics.FromImage(image);
            g.Clear(Color.White);

            int h = pcb.Height / N;
            int w = pcb.Width / M;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    Rectangle rect = new Rectangle(j * w, i * h, w, h);
                    g.FillRectangle(brushes[gridMap.map[i][j]], rect);
                }
            }

            int d = 10; // 直径

            Rectangle origin = IndexInWhichRect(gridMap.origin);
            origin.X -= d / 2;
            origin.Y -= d / 2;
            origin.Width = d;
            origin.Height = d;
            g.FillEllipse(Brushes.Tomato, origin);

            Rectangle goal = IndexInWhichRect(gridMap.goal);
            goal.X -= d / 2;
            goal.Y -= d / 2;
            goal.Width = d;
            goal.Height = d;
            g.FillEllipse(Brushes.Orange, goal);

            //foreach (KDNode goal in gridMap.goals)
            //{
            //    int d = 10;
            //    Rectangle origin = IndexInWhichRect(goal.Node2Point());
            //    origin.X -= d / 2;
            //    origin.Y -= d / 2;
            //    origin.Width = d;
            //    origin.Height = d;
            //    g.FillEllipse(Brushes.Purple, origin);
            //}

            return g;
        }
        public Graphics PaintGraph(Graphics g)  // 画出图结构
        {
            for (int k = 0; k < gridMap.graph.nodes.Count; k++)
            {
                Node node = gridMap.graph.nodes[k];
                for (int i = 0; i < node.neighbor.Count; i++)
                {
                    g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
                                GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
                                GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
                                );
                }
                //int d = 8;
                //Rectangle origin = IndexInWhichRect(gridMap.graph.nodes[k].Node2Point());
                //origin.X -= d / 2;
                //origin.Y -= d / 2;
                //origin.Width = d;
                //origin.Height = d;
                //g.FillEllipse(Brushes.LightBlue, origin);
            }
            return g;
        }

        public Graphics PaintKDTree(Graphics g)  // 画出图结构
        {
            for (int k = 0; k < gridMap.kdtree.kdnodes.Count; k++)
            {
                KDNode node = gridMap.kdtree.kdnodes[k];
                for (int i = 0; i < node.neighbor.Count; i++)
                {
                    try
                    {
                        g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
                                    GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
                                    GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
                                    );
                    }
                    catch { }
                }
                //int d = 8;
                //Rectangle origin = IndexInWhichRect(gridMap.graph.nodes[k].Node2Point());
                //origin.X -= d / 2;
                //origin.Y -= d / 2;
                //origin.Width = d;
                //origin.Height = d;
                //g.FillEllipse(Brushes.LightBlue, origin);
            }
            return g;
        }

        public Graphics PaintTree(Graphics g)  // 画出树结构
        {
            for (int k = 0; k < gridMap.originTree.nodes.Count; k++)
            {
                Node node = gridMap.originTree.nodes[k];
                for (int i = 0; i < node.neighbor.Count; i++)
                {
                    g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
                                GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
                                GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
                                );
                }
                //int d = 8;
                //Rectangle origin = IndexInWhichRect(gridMap.originTree.nodes[k].Node2Point());
                //origin.X -= d / 2;
                //origin.Y -= d / 2;
                //origin.Width = d;
                //origin.Height = d;
                //g.FillEllipse(Brushes.LightBlue, origin);
            }

            for (int k = 0; k < gridMap.goalTree.nodes.Count; k++)
            {
                Node node = gridMap.goalTree.nodes[k];
                for (int i = 0; i < node.neighbor.Count; i++)
                {
                    g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
                                GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
                                GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
                                );
                }
                //int d = 8;
                //Rectangle origin = IndexInWhichRect(gridMap.goalTree.nodes[k].Node2Point());
                //origin.X -= d / 2;
                //origin.Y -= d / 2;
                //origin.Width = d;
                //origin.Height = d;
                //g.FillEllipse(Brushes.LightBlue, origin);
            }
            return g;
        }

        public override Graphics PaintRoad(Graphics g)
        {
            if (gridMap.road.Count != 0)
            {
                for (int i = 0; i < gridMap.road.Count - 1; i++)
                {
                    g.DrawLine(new Pen(Pens.Purple.Color, 4),
                                GetCenterPoint(IndexInWhichRect(gridMap.road[i])),
                                GetCenterPoint(IndexInWhichRect(gridMap.road[i + 1]))
                                );
                    //int d = 10;
                    //Rectangle point = IndexInWhichRect(gridMap.road[i]);
                    //point.X -= d / 2;
                    //point.Y -= d / 2;
                    //point.Width = d;
                    //point.Height = d;
                    //g.FillEllipse(Brushes.Purple, point);
                }
            }

            for (int k = 0; k < gridMap.roads.Count; k++)
            {
                if (gridMap.roads[k].Count != 0)
                {
                    for (int i = 0; i < gridMap.roads[k].Count - 1; i++)
                    {
                        g.DrawLine(new Pen(Pens.Pink.Color, 4),
                                    GetCenterPoint(IndexInWhichRect(gridMap.roads[k][i])),
                                    GetCenterPoint(IndexInWhichRect(gridMap.roads[k][i + 1]))
                                    );
                        //int d = 10;
                        //Rectangle origin = IndexInWhichRect(gridMap.roads[k][i]);
                        //origin.X -= d / 2;
                        //origin.Y -= d / 2;
                        //origin.Width = d;
                        //origin.Height = d;
                        //g.FillEllipse(Brushes.Pink, origin);
                    }
                    int d = 10;
                    Rectangle origin = IndexInWhichRect(gridMap.roads[k][0]);
                    origin.X -= d / 2;
                    origin.Y -= d / 2;
                    origin.Width = d;
                    origin.Height = d;
                    g.FillEllipse(Brushes.Purple, origin);
                    origin = IndexInWhichRect(gridMap.roads[k][gridMap.roads[k].Count - 1]);
                    origin.X -= d / 2;
                    origin.Y -= d / 2;
                    origin.Width = d;
                    origin.Height = d;
                    g.FillEllipse(Brushes.Purple, origin);
                }


            }


            return g;
        }

        public override void pcb_MouseClick(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;
            // 查询当前点所在矩形的索引
            Point p = PointInWhichRectIndex(new Point(ex, ey));
            //// 更改当前栅格的状态
            //gridMap.map[p.X][p.Y] = gridMap.map[p.X][p.Y] == 1 ? 0 : 1; ;
            //// 保存地图
            //gridMap.SaveMap();
            Console.WriteLine(p.X + ", " + p.Y);
        }


    }


}

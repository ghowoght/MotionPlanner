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

            //int d = 10; // 直径

            //Rectangle origin = IndexInWhichRect(gridMap.origin);
            //origin.X -= d / 2;
            //origin.Y -= d / 2;
            //origin.Width = d;
            //origin.Height = d;
            //g.FillEllipse(Brushes.Tomato, origin);

            //Rectangle goal = IndexInWhichRect(gridMap.goal);
            //goal.X -= d / 2;
            //goal.Y -= d / 2;
            //goal.Width = d;
            //goal.Height = d;
            //g.FillEllipse(Brushes.Orange, goal);

            foreach (KDNode goal in gridMap.goals)
            {
                int d = 10;
                Rectangle origin = IndexInWhichRect(goal.Node2Point());
                origin.X -= d / 2;
                origin.Y -= d / 2;
                origin.Width = d;
                origin.Height = d;
                g.FillEllipse(Brushes.Purple, origin);
            }

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


}

    //static class SamplingPainter_
    //{
    //    static private PictureBox pcb;
    //    static private GridMap gridMap;


    //    private static Brush[] brushes = new Brush[] {  Brushes.White,
    //                                                    Brushes.Black,
    //                                                    Brushes.LightBlue,
    //                                                    Brushes.LightCyan,
    //                                                    Brushes.Pink
    //                                                    };

    //    private static int N = 20;
    //    private static int M = 40;


    //    public static void PainterInit(PictureBox pcb_, GridMap map_) // 初始化Painter 
    //    {
    //        N = map_.Height;
    //        M = map_.Width;
    //        pcb = pcb_;
    //        gridMap = map_;
    //        int height = pcb.Height;
    //        //Console.WriteLine(height);
    //        int width = pcb.Width;
    //        //Console.WriteLine(width);
    //        Bitmap image = new Bitmap(width, height);
    //        Graphics g = PaintMap(image);

    //        pcb.Image = image;

    //        new Thread(PainterRefresh).Start();
    //    }
    //    public static void pcb_MouseClick(object sender, MouseEventArgs e)
    //    {
    //        int ex = e.X;
    //        int ey = e.Y;
    //        Point p = PointInWhichRectIndex(new Point(ex, ey)); // 查询当前点所在矩形的索引
    //        gridMap.map[p.X][p.Y] = gridMap.map[p.X][p.Y] == 1 ? 0 : 1;//(gridMap.map[p.X, p.Y] + 1) % 2; // 更改当前栅格的状态
    //        gridMap.SaveMap(); // 保存地图
    //    }

    //    public static Rectangle PointInWhichRect(Point p) // 返回输入点所在的矩形
    //    {
    //        Rectangle rect = new Rectangle(new Point(0, 0), new Size(0, 0));
    //        int h = pcb.Height / N;
    //        int w = pcb.Width / M;
    //        for (int i = 0; i < N; i++)
    //        {
    //            for (int j = 0; j < M; j++)
    //            {
    //                rect = new Rectangle(j * w, i * h, w, h);
    //                if (rect.Contains(p.X, p.Y))
    //                {
    //                    return rect;
    //                }

    //            }
    //        }
    //        return rect;
    //    }
    //    public static Point PointInWhichRectIndex(Point p) // 返回输入点所在的矩形索引
    //    {
    //        Rectangle rect;
    //        int h = pcb.Height / N;
    //        int w = pcb.Width / M;
    //        for (int i = 0; i < N; i++)
    //        {
    //            for (int j = 0; j < M; j++)
    //            {
    //                rect = new Rectangle(j * w, i * h, w, h);
    //                if (rect.Contains(p.X, p.Y))
    //                {
    //                    return new Point(i, j);
    //                }

    //            }
    //        }
    //        return new Point(0, 0);
    //    }
    //    public static Rectangle IndexInWhichRect(Point p) // 返回输入索引所在的矩形
    //    {
    //        int h = pcb.Height / N;
    //        int w = pcb.Width / M;
    //        return new Rectangle(p.Y * w, p.X * h, w, h);
    //    }
    //    public static Point GetCenterPoint(Rectangle rect) // 返回输入索引所在的矩形
    //    {

    //        return new Point(rect.X + rect.Height / 2, rect.Y + rect.Width / 2);
    //    }
    //    public static void pcb_MouseMove(object sender, MouseEventArgs e)
    //    {
    //        int ex = e.X;
    //        int ey = e.Y;

    //        //Bitmap image = new Bitmap(pcb.Image);
    //        //Graphics g = PaintMap(image);

    //        //g.FillRectangle(Brushes.Gray, PointInWhichRect(new Point(ex, ey)));

    //        //pcb.Image = image;
    //    }

    //    public static void PainterRefresh() // 
    //    {
    //        while (true)
    //        {
    //            Bitmap image = new Bitmap(pcb.Image);
    //            // 画出地图
    //            Graphics g = PaintMap(image);
    //            // 画出图结构
    //            g = PaintGraph(g);
    //            // 画出树结构
    //            g = PaintTree(g);
    //            // 画出路径
    //            g = PaintRoad(g);
    //            pcb.Image = image;
    //            Thread.Sleep(50);
    //        }
    //    }

    //    public static Graphics PaintMap(Bitmap image)
    //    {
    //        Graphics g = Graphics.FromImage(image);
    //        g.Clear(Color.White);

    //        int h = pcb.Height / N;
    //        int w = pcb.Width / M;
    //        for (int i = 0; i < N; i++)
    //        {
    //            for (int j = 0; j < M; j++)
    //            {
    //                Rectangle rect = new Rectangle(j * w, i * h, w, h);
    //                g.FillRectangle(brushes[gridMap.map[i][j]], rect);
    //            }
    //        }

    //        int d = 10; // 直径

    //        Rectangle origin = IndexInWhichRect(gridMap.origin);
    //        origin.X -= d / 2;
    //        origin.Y -= d / 2;
    //        origin.Width = d;
    //        origin.Height = d;
    //        g.FillEllipse(Brushes.Tomato, origin);

    //        Rectangle goal = IndexInWhichRect(gridMap.goal);
    //        goal.X -= d / 2;
    //        goal.Y -= d / 2;
    //        goal.Width = d;
    //        goal.Height = d;
    //        g.FillEllipse(Brushes.Orange, goal);

    //        return g;
    //    }

    //    public static Graphics PaintGraph(Graphics g)  // 画出图结构
    //    {
    //        for (int k = 0; k < gridMap.graph.nodes.Count; k++)
    //        {
    //            Node node = gridMap.graph.nodes[k];
    //            for (int i = 0; i < node.neighbor.Count; i++)
    //            {
    //                g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
    //                            GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
    //                            GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
    //                            );
    //            }
    //            int d = 8;
    //            Rectangle origin = IndexInWhichRect(gridMap.graph.nodes[k].Node2Point());
    //            origin.X -= d / 2;
    //            origin.Y -= d / 2;
    //            origin.Width = d;
    //            origin.Height = d;
    //            g.FillEllipse(Brushes.LightBlue, origin);
    //        }
    //        return g;
    //    }

    //    public static Graphics PaintTree(Graphics g)  // 画出树结构
    //    {
    //        for (int k = 0; k < gridMap.originTree.nodes.Count; k++)
    //        {
    //            Node node = gridMap.originTree.nodes[k];
    //            for (int i = 0; i < node.neighbor.Count; i++)
    //            {
    //                g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
    //                            GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
    //                            GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
    //                            );
    //            }
    //            int d = 8;
    //            Rectangle origin = IndexInWhichRect(gridMap.originTree.nodes[k].Node2Point());
    //            origin.X -= d / 2;
    //            origin.Y -= d / 2;
    //            origin.Width = d;
    //            origin.Height = d;
    //            g.FillEllipse(Brushes.LightBlue, origin);
    //        }

    //        for (int k = 0; k < gridMap.goalTree.nodes.Count; k++)
    //        {
    //            Node node = gridMap.goalTree.nodes[k];
    //            for (int i = 0; i < node.neighbor.Count; i++)
    //            {
    //                g.DrawLine(new Pen(Pens.LightBlue.Color, 2),
    //                            GetCenterPoint(IndexInWhichRect(new Point(node.x, node.y))),
    //                            GetCenterPoint(IndexInWhichRect(new Point(node.neighbor[i].x, node.neighbor[i].y)))
    //                            );
    //            }
    //            int d = 8;
    //            Rectangle origin = IndexInWhichRect(gridMap.goalTree.nodes[k].Node2Point());
    //            origin.X -= d / 2;
    //            origin.Y -= d / 2;
    //            origin.Width = d;
    //            origin.Height = d;
    //            g.FillEllipse(Brushes.LightBlue, origin);
    //        }
    //        return g;
    //    }
    //    public static Graphics PaintRoad(Graphics g)
    //    {
    //        if (gridMap.road.Count != 0)
    //        {
    //            for (int i = 0; i < gridMap.road.Count - 1; i++)
    //            {
    //                g.DrawLine(new Pen(Pens.Purple.Color, 4),
    //                            GetCenterPoint(IndexInWhichRect(gridMap.road[i])),
    //                            GetCenterPoint(IndexInWhichRect(gridMap.road[i + 1]))
    //                            );
    //                int d = 10;
    //                Rectangle origin = IndexInWhichRect(gridMap.road[i]);
    //                origin.X -= d / 2;
    //                origin.Y -= d / 2;
    //                origin.Width = d;
    //                origin.Height = d;
    //                g.FillEllipse(Brushes.Purple, origin);
    //            }
    //        }

    //        return g;
    //    }
    //}
}

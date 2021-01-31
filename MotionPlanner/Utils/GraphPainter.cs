using AnimatedGif;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace MotionPlanner
{
    /// <summary>
    /// 针对图搜索算法的画板类
    /// </summary>
    public class GraphPainter : BasePainter
    {

        public GraphPainter(PictureBox pcb_, GridMap map_) // 初始化Painter 
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
            String time = DateTime.Now.ToString("yyyy-MM-dd_hh-mm-ss");
            AnimatedGifCreator agc = new AnimatedGifCreator("img/img_" + time + ".gif");
            agc.AddFrameAsync((Image)image.Clone(), 1000);

            while (true)
            {
                // 画出地图
                g = PaintMap(g);
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
        }

        public override Graphics PaintMap(Graphics g)
        {
            //g.Clear(Color.White);

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
            g.FillRectangle(Brushes.Tomato, IndexInWhichRect(gridMap.origin));
            g.FillRectangle(Brushes.Orange, IndexInWhichRect(gridMap.goal));

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
                }
            }
            return g;
        }

    }
}

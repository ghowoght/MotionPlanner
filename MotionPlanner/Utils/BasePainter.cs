using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.Threading;

namespace MotionPlanner
{
    public abstract class BasePainter
    {

        protected Brush[] brushes = new Brush[] {   Brushes.White,
                                                    Brushes.Black,
                                                    Brushes.LightBlue,
                                                    Brushes.LightCyan,
                                                    Brushes.Pink
                                                    };

        protected int N = 0;
        protected int M = 0;

        protected PictureBox pcb; // 画板
        protected GridMap gridMap; // 地图

        public Rectangle PointInWhichRect(Point p) // 返回输入点所在的矩形
        {
            Rectangle rect = new Rectangle(new Point(0, 0), new Size(0, 0));
            int h = pcb.Height / N;
            int w = pcb.Width / M;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    rect = new Rectangle(j * w, i * h, w, h);
                    if (rect.Contains(p.X, p.Y))
                    {
                        return rect;
                    }

                }
            }
            return rect;
        }
        public Point PointInWhichRectIndex(Point p) // 返回输入点所在的矩形索引
        {
            Rectangle rect;
            int h = pcb.Height / N;
            int w = pcb.Width / M;
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    rect = new Rectangle(j * w, i * h, w, h);
                    if (rect.Contains(p.X, p.Y))
                    {
                        return new Point(i, j);
                    }

                }
            }
            return new Point(0, 0);
        }
        public Rectangle IndexInWhichRect(Point p) // 返回输入索引所在的矩形
        {
            int h = pcb.Height / N;
            int w = pcb.Width / M;
            return new Rectangle(p.Y * w, p.X * h, w, h);
        }
        public Point GetCenterPoint(Rectangle rect) // 返回输入索引所在的矩形
        {
            return new Point(rect.X + rect.Height / 2, rect.Y + rect.Width / 2);
        }
        public void pcb_MouseMove(object sender, MouseEventArgs e) { }

        public abstract void PainterRefresh();
        public abstract Graphics PaintMap(Graphics g);
        public abstract Graphics PaintRoad(Graphics g);

        public abstract void pcb_MouseClick(object sender, MouseEventArgs e);

    }



   

        
}

using System;
using System.Collections.Generic;
using System.Drawing;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace MotionPlanner
{
    static class Painter
    {
        
        private static Brush[] brushes = new Brush[] { Brushes.White, Brushes.Black };

        private static int N = 20;
        private static int M = 40;

        private static PictureBox pcb;
        private static GridMap gridMap;

        public static void PainterInit(PictureBox pcb_, GridMap map_)
        {
            pcb = pcb_;
            gridMap = map_;
            int height = pcb.Height;
            int width = pcb.Width;
            Bitmap image = new Bitmap(width, height);
            Graphics g = Graphics.FromImage(image);
            g.Clear(Color.White);

            int h = height / N;
            int w = width / M;

            for(int i = 0; i < N; i++)
            {
                for(int j = 0; j < M; j++)
                {
                    g.FillRectangle(Brushes.White, new Rectangle(j * w, i * h, w, h));
                }
            }
            pcb.Image = image;
            
        }
        public static void pcb_MouseClick(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;
            Point p = InWhichPoint(new Point(ex, ey));

            gridMap.map[p.X, p.Y] = (gridMap.map[p.X, p.Y] + 1) % 2;

            gridMap.SaveMap("./map.txt");
        }

        public static Rectangle InWhichRect(Point p) // 返回输入点所在的矩形
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
        public static Point InWhichPoint(Point p) // 返回输入点所在的矩形索引
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
        public static void pcb_MouseMove(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;

            Bitmap image = new Bitmap(pcb.Image);
            Graphics g = PaintMap(image);

            g.FillRectangle(Brushes.Gray, InWhichRect(new Point(ex, ey)));

            pcb.Image = image;
        }
        public static void PaintMap(GridMap grid_map_)
        {
            gridMap = grid_map_;
            Bitmap image = new Bitmap(pcb.Image);
            Graphics g = Graphics.FromImage(image);
            int h = pcb.Height / N;
            int w = pcb.Width / M;
            g.Clear(Color.White);
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    Rectangle rect = new Rectangle(j * w, i * h, w, h);
                    g.FillRectangle(brushes[gridMap.map[i, j]], rect);
                    if (rect.Contains(gridMap.origin))
                    {

                        g.FillRectangle(Brushes.Orange, rect);
                    }
                    if (rect.Contains(gridMap.goal))
                    {
                        g.FillRectangle(Brushes.OrangeRed, rect);
                    }
                }
            }
            pcb.Image = image;
        }

        public static Graphics PaintMap(Bitmap image)
        {
            Graphics g = Graphics.FromImage(image);
            g.Clear(Color.White);

            int h = pcb.Height / N;
            int w = pcb.Width / M;            
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    Rectangle rect = new Rectangle(j * w, i * h, w, h);
                    g.FillRectangle(brushes[gridMap.map[i, j]], rect);
                }
            }
            g.FillRectangle(Brushes.Orange, InWhichRect(gridMap.origin));
            g.FillRectangle(Brushes.OrangeRed, InWhichRect(gridMap.goal));
            return g;
        }
    }
}

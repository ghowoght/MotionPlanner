using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace MotionPlanner
{
    static class Painter
    {
        //定义画笔
        private static Pen greenPen = new Pen(Color.Green, 1);
        private static Pen redPen = new Pen(Color.Red, 1);
        private static Pen blackPen = new Pen(Color.Black, 1);
        private static Pen grayPen = new Pen(Color.Gray, 1);
        
        private static Brush[] brushes = new Brush[] { Brushes.White, Brushes.Black };

        private static int N = 20;
        private static int M = 40;

        private static PictureBox pcb;
        private static GridMap grid_map;

        public static void Paint(PictureBox pcb_)
        {
            pcb = pcb_;
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
            Console.WriteLine(ex + " " + ey);
        }
        public static void pcb_MouseMove(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;

            //Bitmap image = new Bitmap(pcb.Width, pcb.Height);
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
                    g.FillRectangle(brushes[grid_map.map[i, j]], rect);
                    if (rect.Contains(ex, ey))
                    {
                        g.FillRectangle(Brushes.Red, rect);
                    }

                }
            }
            pcb.Image = image;
        }
        public static void PaintMap(GridMap grid_map_)
        {
            grid_map = grid_map_;
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
                    g.FillRectangle(brushes[grid_map.map[i, j]], rect);
                }
            }
            pcb.Image = image;
        }
    }
}

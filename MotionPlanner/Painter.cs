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
        
        private static int N = 20;
        private static int M = 40;

        

        public static void Paint(PictureBox pcb)
        {
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
                    g.FillRectangle(Brushes.Blue, new Rectangle(j * w, i * h, w, h));
                }
            }
            pcb.Image = image;

            
        }
        public static void pcb_MouseClick(PictureBox pcb, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;
            Console.WriteLine(ex + " " + ey);
            
        }
        public static void pcb_MouseMove(PictureBox pcb, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;

            Bitmap image = new Bitmap(800, 400);
            Graphics g = Graphics.FromImage(image);
            int height = 400;
            int width = 800;
            int h = height / N;
            int w = width / M;
            g.Clear(Color.White);
            for (int i = 0; i < N; i++)
            {
                for (int j = 0; j < M; j++)
                {
                    Rectangle rect = new Rectangle(j * w, i * h, w, h);
                    if (rect.Contains(ex, ey))
                    {
                        g.FillRectangle(Brushes.Red, rect);
                        g.DrawRectangle(blackPen, rect);
                        Console.WriteLine(ex + " " + ey);
                    }
                }
            }
            pcb.Image = image;

        }
    }
}

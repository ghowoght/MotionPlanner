﻿using System;
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
    static class Painter
    {
        
        private static Brush[] brushes = new Brush[] {  Brushes.White, 
                                                        Brushes.Black, 
                                                        Brushes.LightBlue,
                                                        Brushes.LightCyan,
                                                        Brushes.Pink
                                                        };

        private static int N = 20;
        private static int M = 40;

        private static PictureBox pcb;
        private static GridMap gridMap;

        public static void PainterInit(PictureBox pcb_, GridMap map_) // 初始化Painter 
        {
            N = map_.Height;
            M = map_.Width;
            pcb = pcb_;
            gridMap = map_;
            int height = pcb.Height;
            int width = pcb.Width;
            Bitmap image = new Bitmap(width, height);
            Graphics g = PaintMap(image);
            
            pcb.Image = image;

            new Thread(PainterRefresh).Start();
        }
        public static void pcb_MouseClick(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;
            Point p = PointInWhichRectIndex(new Point(ex, ey)); // 查询当前点所在矩形的索引
            gridMap.map[p.X][p.Y] = gridMap.map[p.X][p.Y] == 1 ? 0 : 1;//(gridMap.map[p.X, p.Y] + 1) % 2; // 更改当前栅格的状态
            gridMap.SaveMap(); // 保存地图
        }

        public static Rectangle PointInWhichRect(Point p) // 返回输入点所在的矩形
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
        public static Point PointInWhichRectIndex(Point p) // 返回输入点所在的矩形索引
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
        public static Rectangle IndexInWhichRect(Point p) // 返回输入索引所在的矩形
        {
            int h = pcb.Height / N;
            int w = pcb.Width / M;
            return new Rectangle(p.Y * w, p.X * h, w, h);
        }
        public static Point GetCenterPoint(Rectangle rect) // 返回输入索引所在的矩形
        {

            return new Point(rect.X + rect.Height / 2, rect.Y + rect.Width / 2);
        }
        public static void pcb_MouseMove(object sender, MouseEventArgs e)
        {
            int ex = e.X;
            int ey = e.Y;

            //Bitmap image = new Bitmap(pcb.Image);
            //Graphics g = PaintMap(image);

            //g.FillRectangle(Brushes.Gray, PointInWhichRect(new Point(ex, ey)));

            //pcb.Image = image;
        }

        public static void PainterRefresh() // 
        {
            while (true)
            {
                Bitmap image = new Bitmap(pcb.Image);
                Graphics g = PaintMap(image);
                g = PaintRoad(g);
                pcb.Image = image;
                Thread.Sleep(50);
            }
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
                    g.FillRectangle(brushes[gridMap.map[i][j]], rect);
                }
            }
            g.FillRectangle(Brushes.Tomato, IndexInWhichRect(gridMap.origin));
            g.FillRectangle(Brushes.Orange, IndexInWhichRect(gridMap.goal));

            return g;
        }
        public static Graphics PaintRoad(Graphics g)
        {
            if (gridMap.road.Count != 0)
            {
                for (int i = 0; i < gridMap.road.Count - 1; i++)
                {
                    g.DrawLine( new Pen(Pens.Purple.Color, 4), 
                                GetCenterPoint(IndexInWhichRect(gridMap.road[i])), 
                                GetCenterPoint(IndexInWhichRect(gridMap.road[i + 1]))
                                );
                }
            }

            return g;
        }
    }
}

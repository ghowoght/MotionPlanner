using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MotionPlanner
{
    class InformedRRT : SamplingPlanner
    {
        const int ITER = 10000;                      // 迭代次数
        const int NUM_SAMPLES = 20000;              // 采样点数
        const double MAX_DISTANCE = 30;             // 两个节点建立连接的最大距离
        const double RandomProbability = 0.99;      // 随机采样概率
        public Graph samplesGraph = new Graph();    // 采样后的样点图
        

        public InformedRRT(GridMap map)
        {
            this.map = map;
            map.graph = samplesGraph;
        }
        public Node RotationToWorldFrame(Node sample, Node center, double theta)
        {
            Node sample_new = new Node( (int)(Math.Cos(theta) * sample.x + Math.Sin(theta) * sample.y + center.x),
                                        (int)(Math.Cos(theta) * sample.y - Math.Sin(theta) * sample.x + center.y));
            sample_new.x = sample_new.x < 0 ? 0 : sample_new.x;
            sample_new.x = sample_new.x >= map.Height ? (map.Height - 1) : sample_new.x;
            sample_new.y = sample_new.y < 0 ? 0 : sample_new.y;
            sample_new.y = sample_new.y >= map.Width ? (map.Width - 1) : sample_new.y;

            return sample_new;
        }
        public Node GetSample(Node origin, Node goal, double c_max, Random rnd)
        {
            Node sample = new Node(rnd.Next(0, map.Height), rnd.Next(0, map.Width));
            if (c_max == double.MaxValue)
            {
                while (map.map[sample.x][sample.y] == (int)GridMap.MapStatus.Occupied)
                {
                    sample = new Node(rnd.Next(0, map.Height), rnd.Next(0, map.Width));
                }
                return sample;
            }
            double x_max = c_max / 2; // 长轴
            
            double c = GetEuclideanDistance(goal, origin) / 2.0; // 焦点坐标
            double y_max = Math.Sqrt(x_max * x_max + c * c); // 短轴
            int x = rnd.Next((int)-x_max, (int)x_max);
            int y = rnd.Next((int)-y_max, (int)y_max);
            Node sample0 = new Node(x, y);
            
            do
            {
                // 判断采样点是否在椭圆内
                // 如果采样点到两焦点的距离之和小于2a，则采样点在椭圆内
                while (GetEuclideanDistance(sample0, new Node((int)-c, 0)) + GetEuclideanDistance(sample, new Node((int)c, 0)) > 2 * x_max)
                {
                    sample0 = new Node(rnd.Next((int)-x_max, (int)x_max),
                                        rnd.Next((int)-y_max, (int)y_max));
                }
                //double theta = Math.Atan2(goal.y - origin.y, goal.x - origin.x);
                sample = RotationToWorldFrame(  sample0,
                                                new Node((origin.x + goal.x) / 2, (origin.y + goal.y) / 2),
                                                Math.Atan2(goal.y - origin.y, goal.x - origin.x));
            } while (map.map[sample.x][sample.y] == (int)GridMap.MapStatus.Occupied);
            return sample;
        }

        public void Search()
        {
            Node origin = new Node(map.origin.X, map.origin.Y); // 起点
            Node goal = new Node(map.goal.X, map.goal.Y);       // 终点

            samplesGraph.nodes.Add(origin); // 将起点加入采样图

            Random rnd = new Random();

            double c_best = double.MaxValue;
            for(int iter = 0; iter < ITER; iter++)
            {
                Node sample0 = GetSample(origin, goal, c_best, rnd); // 保存原始采样点

                // 初始化采样点为原始采样点
                Node sample = new Node(sample0.x, sample0.y);
                // 将已知样点集的第一个节点作为距离其最近的节点
                sample.front = samplesGraph.nodes[0];
                double distance_near = GetEuclideanDistance(samplesGraph.nodes[0], sample);

                // 寻找已采样点集中与采样点距离最近且无碰撞的节点
                bool flag = false;
                for (int i = 0; i < samplesGraph.nodes.Count; i++)
                {

                    Node temp = new Node(sample0.x, sample0.y);

                    if (GetEuclideanDistance(samplesGraph.nodes[i], sample0) > MAX_DISTANCE)
                    {
                        // 如果原始采样点与当前节点距离大于最大距离
                        // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                        // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                        double theta = Math.Atan2(sample0.y - samplesGraph.nodes[i].y, sample0.x - samplesGraph.nodes[i].x); // 倾角
                        temp = new Node(samplesGraph.nodes[i].x + (int)(MAX_DISTANCE * Math.Cos(theta)),
                                            samplesGraph.nodes[i].y + (int)(MAX_DISTANCE * Math.Sin(theta)));
                    }
                    temp.front = sample.front;

                    if (!isCollision(samplesGraph.nodes[i], temp)) // 如果采样点和当前点间连线没有碰撞
                    {
                        flag = true;
                        double distance = GetEuclideanDistance(samplesGraph.nodes[i], sample0);
                        if (distance < distance_near)
                        {
                            temp.front = samplesGraph.nodes[i];
                            sample = temp;
                            distance_near = distance;
                        }
                        if (sample.front.isEqual(origin))
                        {
                            temp.front = samplesGraph.nodes[i];
                            sample = temp;
                        }

                    }
                }
                if (flag) // 如果采样点和已知样点图间存在一条无碰撞路径
                {
                    samplesGraph.nodes.Add(sample);
                    sample.front.neighbor.Add(sample);

                    // 到达终点后，回溯得到路径
                    if (GetEuclideanDistance(sample, goal) < MAX_DISTANCE)
                    {
                        double c_new = 0;
                        map.road.Add(goal.Node2Point());
                        c_new += GetEuclideanDistance(sample, goal);
                        while (sample.front != null)
                        {
                            map.map[sample.x][sample.y] = (int)GridMap.MapStatus.Road;
                            map.road.Add(new Point(sample.x, sample.y));
                            c_new += GetEuclideanDistance(sample, sample.front);
                            sample = sample.front;
                        }
                        c_new += GetEuclideanDistance(sample, origin);
                        c_best = c_new;
                        Console.WriteLine("c_best: " + c_best);
                        map.road.Add(map.origin);
                        //break;
                    }
                }
            }

            Thread.Sleep(100);
            map.searchFlag = 1;

        }
    }
}

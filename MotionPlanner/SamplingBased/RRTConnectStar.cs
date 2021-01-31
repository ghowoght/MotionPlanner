using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;

namespace MotionPlanner
{
    class RRTConnectStar
    {

        const int NUM_SAMPLES = 20000; // 采样点数
        const double MAX_DISTANCE = 5; // 两个节点建立连接的最大距离
        const double OptimizationR = 50;  // 优化半径
        const double RandomProbability = 0.99; // 随机采样概率
        public Tree originTree = new Tree(); // 以起点作为根节点的树
        public Tree goalTree = new Tree(); // 以终点作为根节点的树
        GridMap map;
        public RRTConnectStar(GridMap map)
        {
            this.map = map;
            map.originTree = originTree;
            map.goalTree = goalTree;
        }

        /// <summary>
        /// 搜索路径函数
        /// </summary>
        public void Search()
        {
            Thread.Sleep(1000);
            Node origin = new Node(map.origin.X, map.origin.Y); // 起点
            Node goal = new Node(map.goal.X, map.goal.Y); // 终点

            originTree.nodes.Add(origin); // 将起点加入采样图
            goalTree.nodes.Add(goal);

            while (originTree.nodes.Count < NUM_SAMPLES)
            {
                // 获取原始采样点
                Node sample0 = GetSample();

                // 对以起点为根节点的树进行操作

                if (map.map[sample0.x][sample0.y] != (int)GridMap.MapStatus.Occupied)
                {
                    // 初始化采样点为原始采样点
                    Node sample = new Node(sample0.x, sample0.y);
                    // 将已知样点集的第一个节点作为距离其最近的节点
                    sample.front = originTree.nodes[0];
                    double distance_near = GetEuclideanDistance(originTree.nodes[0], sample);

                    // 寻找已采样点集中与采样点距离最近且无碰撞的节点
                    bool flag = false;
                    for (int i = 0; i < originTree.nodes.Count; i++)
                    {

                        Node temp = new Node(sample0.x, sample0.y);

                        if (GetEuclideanDistance(originTree.nodes[i], sample0) > MAX_DISTANCE)
                        {
                            // 如果原始采样点与当前节点距离大于最大距离
                            // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                            // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                            double theta = Math.Atan2(sample0.y - originTree.nodes[i].y, sample0.x - originTree.nodes[i].x); // 倾角
                            temp = new Node(originTree.nodes[i].x + (int)(MAX_DISTANCE * Math.Cos(theta)),
                                                originTree.nodes[i].y + (int)(MAX_DISTANCE * Math.Sin(theta)));
                        }
                        temp.front = sample.front;

                        if (!isCollision(originTree.nodes[i], temp)) // 如果采样点和当前点间连线没有碰撞
                        {
                            flag = true;
                            double distance = GetEuclideanDistance(originTree.nodes[i], sample0);
                            if (distance < distance_near)
                            {
                                temp.front = originTree.nodes[i];
                                sample = temp;
                                distance_near = distance;
                            }
                            if (sample.front.isEqual(origin))
                            {
                                temp.front = originTree.nodes[i];
                                sample = temp;
                            }

                        }
                    }
                    if (flag) // 如果采样点和已知样点图间存在一条无碰撞路径
                    {
                        originTree.nodes.Add(sample);
                        sample.front.neighbor.Add(sample);

                        sample.cost = sample.front.cost + GetEuclideanDistance(sample, sample.front); // 计算代价

                        // 优化路径
                        double R = OptimizationR;
                        foreach (Node n in originTree.nodes) // 遍历所有潜在父节点
                        {
                            double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                            if (sample.cost > cost) // 如果新路径代价更小
                            {
                                if (!isCollision(sample, n, R)) // 碰撞检测通过
                                {
                                    sample.front.Remove(sample); // 将之前的边删掉
                                    sample.cost = cost; // 更新代价
                                    sample.front = n; // 更新路径
                                    sample.front.neighbor.Add(sample); // 添加新边
                                }
                            }

                        }

                        Node near = goalTree.nodes.Last();
                        double near_cost = near.cost + GetEuclideanDistance(near, sample);
                        foreach (Node n in goalTree.nodes) // 遍历所有潜在父节点
                        {
                            double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                            if (near_cost > cost) // 如果新路径代价更小
                            {
                                if (!isCollision(n, sample, MAX_DISTANCE))
                                {
                                    near = n;
                                    near_cost = cost;
                                }

                            }
                        }
                        // 两棵树交汇后，回溯得到路径
                        if (!isCollision(near, originTree.nodes.Last(), MAX_DISTANCE))
                        {

                            List<Node> road = new List<Node>();

                            // 将两棵树进行拼接，得到一条初始路径
                            sample = near;
                            while (sample.front != null)
                            {
                                road.Add(sample);
                                sample = sample.front;
                            }
                            road.Add(goal);

                            sample = originTree.nodes.Last();
                            while (sample.front != null)
                            {
                                road.Insert(0, sample);
                                sample = sample.front;
                            }
                            road.Insert(0, origin);


                            for (int i = 1; i < road.Count(); i++)
                            {
                                road[i].cost = road[i - 1].cost + GetEuclideanDistance(road[i], road[i - 1]);
                                road[i].front = road[i - 1];
                            }

                            // 优化上面得到的路径
                            foreach (Node n1 in road)
                            {
                                foreach (Node n2 in road)
                                {
                                    double cost = n1.cost + GetEuclideanDistance(n1, n2);
                                    if (n2.cost > cost)
                                    {
                                        if (!isCollision(n1, n2, OptimizationR))
                                        {
                                            n2.front = n1;
                                        }
                                    }
                                }
                            }

                            Node temp = road.Last();
                            while (temp.front != null)
                            {
                                map.map[temp.x][temp.y] = (int)GridMap.MapStatus.Road;
                                map.road.Add(new Point(temp.x, temp.y));
                                temp = temp.front;
                            }
                            map.road.Add(new Point(origin.x, origin.y));

                            break;
                        }
                    }
                }

                // 对以终点为根节点的树进行上述操作

                // 重新采样
                double p = rd.Next(0, 100) / 100.0;
                if (p < RandomProbability)
                {
                    sample0 = new Node(rd.Next(0, map.Height), rd.Next(0, map.Width));
                }
                else
                {
                    sample0 = originTree.nodes.Last();
                }


                if (map.map[sample0.x][sample0.y] != (int)GridMap.MapStatus.Occupied)
                {
                    // 初始化采样点为原始采样点
                    Node sample = new Node(sample0.x, sample0.y);
                    // 将已知样点集的第一个节点作为距离其最近的节点
                    sample.front = goalTree.nodes[0];
                    double distance_near = GetEuclideanDistance(goalTree.nodes[0], sample);

                    // 寻找已采样点集中与采样点距离最近且无碰撞的节点
                    bool flag = false;
                    for (int i = 0; i < goalTree.nodes.Count; i++)
                    {

                        Node temp = new Node(sample0.x, sample0.y);

                        if (GetEuclideanDistance(goalTree.nodes[i], sample0) > MAX_DISTANCE)
                        {
                            // 如果原始采样点与当前节点距离大于最大距离
                            // 则将两节点连线上距离当前节点MAX_DISTANCE远的节点作为新采样点
                            // 也就是当前节点朝原始采样点移动MAX_DISTANCE后得到的节点
                            double theta = Math.Atan2(sample0.y - goalTree.nodes[i].y, sample0.x - goalTree.nodes[i].x); // 倾角
                            temp = new Node(goalTree.nodes[i].x + (int)(MAX_DISTANCE * Math.Cos(theta)),
                                                goalTree.nodes[i].y + (int)(MAX_DISTANCE * Math.Sin(theta)));
                        }
                        temp.front = sample.front;

                        if (!isCollision(goalTree.nodes[i], temp)) // 如果采样点和当前点间连线没有碰撞
                        {
                            flag = true;
                            double distance = GetEuclideanDistance(goalTree.nodes[i], sample0);
                            if (distance < distance_near)
                            {
                                temp.front = goalTree.nodes[i];
                                sample = temp;
                                distance_near = distance;
                            }
                            if (sample.front.isEqual(goal))
                            {
                                temp.front = goalTree.nodes[i];
                                sample = temp;
                            }

                        }
                    }
                    if (flag) // 如果采样点和已知样点图间存在一条无碰撞路径
                    {
                        goalTree.nodes.Add(sample);
                        sample.front.neighbor.Add(sample);

                        sample.cost = sample.front.cost + GetEuclideanDistance(sample, sample.front); // 计算代价

                        // 优化路径
                        double R = OptimizationR;
                        foreach (Node n in goalTree.nodes) // 遍历所有潜在父节点
                        {
                            double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                            if (sample.cost > cost) // 如果新路径代价更小
                            {
                                if (!isCollision(sample, n, R)) // 碰撞检测通过
                                {
                                    sample.front.Remove(sample); // 将之前的边删掉
                                    sample.cost = cost; // 更新代价
                                    sample.front = n; // 更新路径
                                    sample.front.neighbor.Add(sample); // 添加新边
                                }
                            }

                        }

                        Node near = originTree.nodes.Last();
                        double near_cost = near.cost + GetEuclideanDistance(near, sample);
                        foreach (Node n in originTree.nodes) // 遍历所有潜在父节点
                        {
                            double cost = n.cost + GetEuclideanDistance(sample, n); // 计算代价
                            if (near_cost > cost) // 如果新路径代价更小
                            {
                                if (!isCollision(n, sample, MAX_DISTANCE))
                                {
                                    near = n;
                                    near_cost = cost;
                                }

                            }
                        }
                        // 两棵树交汇后，回溯得到路径
                        if (!isCollision(near, goalTree.nodes.Last(), MAX_DISTANCE))
                        {
                            
                            List<Node> road = new List<Node>();

                            // 将两棵树进行拼接，得到一条初始路径
                            sample = near;
                            while (sample.front != null)
                            {
                                road.Add(sample);
                                sample = sample.front;
                            }
                            road.Add(origin);

                            sample = goalTree.nodes.Last();
                            while (sample.front != null)
                            {
                                road.Insert(0, sample);
                                sample = sample.front;
                            }
                            road.Insert(0, goal);

                            road.Reverse();

                            for (int i = 1; i < road.Count() ; i++)
                            {
                                road[i].cost = road[i - 1].cost + GetEuclideanDistance(road[i], road[i - 1]);
                                road[i].front = road[i - 1];
                            }
                            
                            // 优化上面得到的路径
                            foreach (Node n1 in road)
                            {
                                foreach(Node n2 in road)
                                {
                                    double cost = n1.cost + GetEuclideanDistance(n1, n2);
                                    if (n2.cost > cost)
                                    {
                                        if (!isCollision(n1, n2, OptimizationR))
                                        {
                                            n2.front = n1;
                                        }
                                    }
                                }
                            }

                            Node temp = road.Last();
                            while (temp.front != null)
                            {
                                map.map[temp.x][temp.y] = (int)GridMap.MapStatus.Road;
                                map.road.Add(new Point(temp.x, temp.y));
                                temp = temp.front;                            
                            }
                            map.road.Add(new Point(origin.x, origin.y));

                            break;
                        }
                    }
                }
                Thread.Sleep(10);

            }
            Thread.Sleep(100);
            map.searchFlag = 1;
        }

        /// <summary>
        /// 随机数种子
        /// </summary>
        Random rd = new Random();
        /// <summary>
        /// 获取采样点
        /// </summary>
        /// <returns></returns>
        public Node GetSample()
        {
            double p = rd.Next(0, 100) / 100.0;
            if (p < RandomProbability)
            {
                return new Node(rd.Next(0, map.Height), rd.Next(0, map.Width));
            }
            else
            {
                return new Node(map.goal.X, map.goal.Y);
            }
        }

        /// <summary>
        /// 计算两个节点间的欧氏距离
        /// </summary>
        /// <param name="n1">节点1</param>
        /// <param name="n2">节点2</param>
        /// <returns></returns>
        private double GetEuclideanDistance(Node n1, Node n2)
        {
            return Math.Sqrt(Math.Pow(n1.x - n2.x, 2) + Math.Pow(n1.y - n2.y, 2));
        }

        /// <summary>
        /// 判断两点间连线是否会经过障碍物，以及两点间距离是否在最大连线范围内
        /// </summary>
        /// <param name="n1">节点1</param>
        /// <param name="n2">节点2</param>
        /// <param name="maxDist">最大距离</param>
        /// <returns></returns>
        public bool isCollision(Node n1, Node n2, double maxDist)
        {

            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x); // 倾角
            double dist = GetEuclideanDistance(n1, n2); // 两点距离
            if (dist > maxDist)
            {
                return true;
            }
            double step = 1; // 步长
            int n_step = (int)(dist / step); // 步数
            PointF p = new PointF((float)n1.x + (float)(step * Math.Cos(theta)),
                                (float)n1.y + (float)(step * Math.Sin(theta)));
            for (int i = 0; i < n_step; i++)
            {
                if (map.map[(int)Math.Round(p.X)][(int)Math.Round(p.Y)] == (int)GridMap.MapStatus.Occupied)
                {
                    return true;  // 有碰撞
                }

                p.X += (float)(step * Math.Cos(theta));
                p.Y += (float)(step * Math.Sin(theta));
            }
            return false;
        }

        /// <summary>
        /// 判断两点间连线是否会经过障碍物
        /// </summary>
        /// <param name="n1">节点1</param>
        /// <param name="n2">节点2</param>
        /// <returns></returns>
        public bool isCollision(Node n1, Node n2)
        {

            double theta = Math.Atan2(n2.y - n1.y, n2.x - n1.x); // 倾角
            double dist = GetEuclideanDistance(n1, n2); // 两点距离

            double step = 1; // 步长
            int n_step = (int)(dist / step); // 步数
            PointF p = new PointF((float)n1.x + (float)(step * Math.Cos(theta)),
                                (float)n1.y + (float)(step * Math.Sin(theta)));
            for (int i = 0; i < n_step; i++)
            {
                if (map.map[(int)Math.Round(p.X)][(int)Math.Round(p.Y)] == (int)GridMap.MapStatus.Occupied)
                {
                    return true;  // 有碰撞
                }

                p.X += (float)(step * Math.Cos(theta));
                p.Y += (float)(step * Math.Sin(theta));
            }
            return false;
        }
    }
}
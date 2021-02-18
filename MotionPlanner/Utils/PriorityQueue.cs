using System.Collections.Generic;

namespace MotionPlanner
{
    internal class PriorityQueue<T>
    {
        public List<T> datas = new List<T>();
        private int count = 0;

        public PriorityQueue(){ }
        public int Count 
        {
            get
            {
                return count;
            }
        }
        public T Top()
        {
            return datas[0];
        }
        public bool Empty()
        {
            return count == 0;
        }
        public void Push(T data) // 将一个数据添加到队尾
        {
            datas.Add(data);
            datas.Sort();
            count++;
        }
        public T Pop() // 从队首取出一个数据
        {
            T data = Top();
            datas.RemoveAt(0);
            count--;
            return data;
        }
        public T this[int i] // 索引
        {
            get
            {
                return datas[i];
            }
            set
            {
                datas[i] = value;
            }
        }

    }
}
using System.Collections.Generic;

namespace MotionPlanner
{
    internal class PriorityQueue<T>
    {
        private List<T> datas = new List<T>();
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
        public void Push(T data)
        {
            datas.Add(data);
            datas.Sort();
            count++;
        }
        public T Pop()
        {
            T data = Top();
            datas.RemoveAt(0);
            count--;
            return data;
        }

    }
}
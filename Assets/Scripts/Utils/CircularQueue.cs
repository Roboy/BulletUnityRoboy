using System.Collections.Generic;
using System.Linq;

namespace Utils
{
    // Courtesy to https://forum.unity.com/threads/circular-buffers.222277/
    public class CircularQueue<T>
    {
        private readonly Queue<T> _queue;
        private readonly int _size;

        public CircularQueue(int size)
        {
            _queue = new Queue<T>(size);
            _size = size;
        }

        public void Add(T obj)
        {
            if (_queue.Count == _size)
            {
                _queue.Dequeue();
                _queue.Enqueue(obj);
            }
            else
                _queue.Enqueue(obj);
        }

        public T Read()
        {
            return _queue.Dequeue();
        }

        public int CountVar(T variable)
        {
            lock (this)
            {
                return _queue.Count(t => Equals(t, variable));
            }
        }

        public T Peek()
        {
            return _queue.Peek();
        }
    }
}
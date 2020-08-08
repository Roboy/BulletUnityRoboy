using System;

namespace Utils
{
    [Serializable]
    class Wrapper<T>
    {
        public T data;

        public Wrapper(T data)
        {
            this.data = data;
        }
    }
}
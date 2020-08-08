using UnityEngine;

namespace Utils
{
    public class ShuffleArray
    {
        public static void Shuffle<T>(T[] arr)
        {
            for (int i = arr.Length - 1; i > 0; i--)
            {
                int r = Random.Range(0, i + 1);
                T tmp = arr[i];
                arr[i] = arr[r];
                arr[r] = tmp;
            }
        }
    }
}
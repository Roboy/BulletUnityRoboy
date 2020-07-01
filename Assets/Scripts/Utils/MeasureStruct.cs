using System;

namespace Utils
{
    /**
     * Struct to perform averaged profiling.
     * Usage:       float tmpTimeC = Time.realtimeSinceStartup;
                    measurePointC.AccumulatedTime += Time.realtimeSinceStartup - tmpTimeC;
                    measurePointC.Count += 1;
     */
    public struct MeasureStruct
    {
        private float _accumulatedTime;
        private float _count;

        public MeasureStruct(float accumulatedTime, float count)
        {
            this._accumulatedTime = accumulatedTime;
            this._count = count;
        }

        public float AccumulatedTime
        {
            get => _accumulatedTime;
            set => _accumulatedTime = value;
        }

        public float Count
        {
            get => _count;
            set => _count = value;
        }

        public String PrintResult()
        {
            return "Time: " + _accumulatedTime / _count;
        }
    }
}
using System;
using UnityEngine;

namespace Controller.Helper.Study
{
    [Serializable]
    public class StudyPersonalLimit
    {
        [SerializeField] private float delay;
        [SerializeField] private float velocity;
        [SerializeField] private int robotId;

        public float Delay
        {
            get => delay;
            set => delay = value;
        }

        public float Velocity
        {
            get => velocity;
            set => velocity = value;
        }

        public int RobotId
        {
            get => robotId;
            set => robotId = value;
        }
    }
}
using System;
using UnityEngine;

namespace Controller.Helper.Study
{
    [Serializable]
    public class StudyState
    {
        [SerializeField] private float velocity;
        [SerializeField] private float delay;
        [SerializeField] private int robotId;

        public StudyState(float velocity, float delay, int robotId)
        {
            this.velocity = velocity;
            this.delay = delay;
            this.robotId = robotId;
        }
    }
}
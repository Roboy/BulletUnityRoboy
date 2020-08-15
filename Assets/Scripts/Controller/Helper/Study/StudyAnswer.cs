using System;
using UnityEngine;

namespace Controller.Helper.Study
{
    [Serializable]
    public class StudyAnswer
    {
        [SerializeField] private int answer;
        [SerializeField] private float embodiment;
        [SerializeField] private int bodyOwnership;
        [SerializeField] private int agency;
        [SerializeField] private int location;

        [SerializeField] private string atTime;

        public int Answer
        {
            get => answer;
            set => answer = value;
        }

        public string AtTime
        {
            get => atTime;
            set => atTime = value;
        }

        public float Embodiment => embodiment;

        public int BodyOwnership => bodyOwnership;

        public int Agency => agency;

        public int Location => location;

        public StudyAnswer(int answer, float embodiment, int bodyOwnership, int agency, int location, string atTime)
        {
            this.answer = answer;
            this.embodiment = embodiment;
            this.bodyOwnership = bodyOwnership;
            this.agency = agency;
            this.location = location;
            this.atTime = atTime;
        }
    }
}
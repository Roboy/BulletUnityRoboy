using System;
using UnityEngine;

namespace Controller.Helper.Study
{
    [Serializable]
    public class StudyAnswer
    {
        [SerializeField] private int answer;

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

        public StudyAnswer(int answer, string atTime)
        {
            Answer = answer;
            AtTime = atTime;
        }
    }
}
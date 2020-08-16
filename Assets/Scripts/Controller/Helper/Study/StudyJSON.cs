using System;
using System.Collections.Generic;
using UnityEngine;

namespace Controller.Helper.Study
{
    [Serializable]
    public class StudyJSON
    {
        [SerializeField] private List<StudyQuestion> studyQuestions;
        [SerializeField] private StudyPersonalLimit studyPersonalLimit;

        public StudyJSON(List<StudyQuestion> studyQuestions, StudyPersonalLimit studyPersonalLimit)
        {
            this.studyQuestions = studyQuestions;
            this.studyPersonalLimit = studyPersonalLimit;
        }
    }
}
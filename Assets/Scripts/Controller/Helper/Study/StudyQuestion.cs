using System;
using UnityEngine;

namespace Controller.Helper.Study
{
    
    [Serializable]
    public class StudyQuestion
    {
        [SerializeField] private String id;
        private string _english;
        private string _german;
        [SerializeField] private StudyAnswer answer;
        [SerializeField] private StudyQuestionCategory category;

        public enum StudyQuestionCategory
        {
            BodyOwnership,
            Agency,
            Location,
        }

        public String Id => id;

        public string English => _english;

        public string German => _german;

        public StudyAnswer Answer
        {
            get => answer;
            set => answer = value;
        }

        public StudyQuestionCategory Category => category;

        public StudyQuestion(String id, string english, string german, StudyQuestionCategory category)
        {
            this.id = id;
            this._english = english;
            this._german = german;
            this.category = category;
        }
    }
}
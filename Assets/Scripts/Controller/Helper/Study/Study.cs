using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEngine;
using Utils;

namespace Controller.Helper.Study
{
    public class Study
    {
        public enum StudyType
        {
            None,
            Delay,
            Velocity,
            JointLimits,
            Combined,
            AllIn
        }

        public StudyType Type => _type;

        public int QuestionIndex { get; set; } = 0;

        public List<StudyQuestion> StudyQuestions => _studyQuestions;

        // Meta
        private readonly DateTime _startTime;
        private readonly String _filePath;
        private readonly StudyController.StudyLanguage _language;
        private readonly LimitationController _limitationController;

        /// <summary>
        /// Object, that holds all questions that can be asked during the study.
        /// </summary>
        private readonly List<StudyQuestion> _allStudyQuestions = new List<StudyQuestion>()
        {
            new StudyQuestion("Q1", "I feel as if the virtual arm is my arm", "Ich habe das Gefühl, dass der virtuelle Arm mein Arm ist", StudyQuestion.StudyQuestionCategory.BodyOwnership),
            new StudyQuestion("Q2", "It feels as if the virtual arm I see is someone else", "Es fühlt sich an, als ob der virtuelle Arm, den ich sehe, jemand anderes ist",
                StudyQuestion.StudyQuestionCategory.BodyOwnership),
            new StudyQuestion("Q3", "It seems as if I might have more than one arm", "Es scheint, als hätte ich mehr als einen Arm", StudyQuestion.StudyQuestionCategory.BodyOwnership),
            new StudyQuestion("Q4", "I feel as if the virtual arm I see when looking in the mirror is my own arm",
                "Ich habe das Gefühl, dass der virtuelle Arm, den ich beim Blick in den Spiegel sehe, mein eigener Arm ist", StudyQuestion.StudyQuestionCategory.BodyOwnership),
            new StudyQuestion("Q5", "I feel as if the virtual arm I see when looking at myself in the mirror is another person",
                "Ich habe das Gefühl, dass der virtuelle Arm, den ich sehe, wenn ich mich im Spiegel betrachte, eine andere Person ist", StudyQuestion.StudyQuestionCategory.BodyOwnership),
            new StudyQuestion("Q6", "It feels like I could control the virtual arm as if it is my own arm", "Es fühlt sich an, als könnte ich den virtuellen Arm kontrollieren, als wäre es mein eigener Arm",
                StudyQuestion.StudyQuestionCategory.Agency),
            new StudyQuestion("Q7", "The movements of the virtual arm are caused by my movements", "Die Bewegungen des virtuellen Arms werden durch meine Bewegungen verursacht",
                StudyQuestion.StudyQuestionCategory.Agency),
            new StudyQuestion("Q8", "I feel as if the movements of the virtual arm are influencing my own movements", "Ich habe das Gefühl, dass die Bewegungen des virtuellen Arms meine eigenen Bewegungen beeinflussen",
                StudyQuestion.StudyQuestionCategory.Agency),
            new StudyQuestion("Q9", "I feel as if the virtual arm is moving by itself", "Ich habe das Gefühl, der virtuelle Arm bewegt sich von selbst", StudyQuestion.StudyQuestionCategory.Agency),
            new StudyQuestion("Q14", "I feel as if my arm is located where I see the virtual arm", "Ich habe das Gefühl, dass sich mein Arm dort befindet, wo ich den virtuellen Arm sehe",
                StudyQuestion.StudyQuestionCategory.Location),
            new StudyQuestion("Q15", "I feel out of my body", "Ich fühle mich, als wäre ich außerhalb meines Körpers", StudyQuestion.StudyQuestionCategory.Location),
            new StudyQuestion("Q16", "I feel as if my (real) arm is drifting toward the virtual arm or as if the virtual arm is drifting toward my (real) arm",
                "Ich habe das Gefühl, als ob mein (realer) Arm auf den virtuellen Arm zusteuert oder als ob der virtuelle Arm auf meinen (realen) Arm zusteuert.", StudyQuestion.StudyQuestionCategory.Location),
        };

        /// <summary>
        /// Ordered list of questions, that are answered during this particular study.
        /// </summary>
        private readonly List<StudyQuestion> _studyQuestions = new List<StudyQuestion>();

        /// <summary>
        /// Type of the current study.
        /// </summary>
        private readonly StudyType _type = StudyType.None;

        public string FilePath => _filePath;

        public StudyPersonalLimit PersonalLimit { get; } = new StudyPersonalLimit();

        public Study(StudyType studyType, StudyController.StudyLanguage language, int studyQuestionsNumber, LimitationController limitationController)
        {
            _type = studyType;

            _startTime = DateTime.Now;
            _language = language;
            _limitationController = limitationController;

            _filePath = StudyController.savePath + "/" + _startTime.ToShortDateString() + "_" + StudyUserData.Instance.Forename + "-" + StudyUserData.Instance.Surname + "_" + StudyUserData.Instance.Age + "_" +
                        StudyUserData.Instance.Uuid + "/" + Type.ToString() + ".json";
            if (!Directory.Exists(Path.GetDirectoryName(_filePath)))
            {
                Directory.CreateDirectory(Path.GetDirectoryName(_filePath));
            }


            foreach (StudyQuestion.StudyQuestionCategory studyQuestionCategory in Enum.GetValues(typeof(StudyQuestion.StudyQuestionCategory)))
            {
                StudyQuestion[] tmpShuffledQuestions = (_allStudyQuestions.Where((question => question.Category == studyQuestionCategory)).ToArray());
                Utils.ShuffleArray.Shuffle(tmpShuffledQuestions);

                foreach (var shuffledQuestion in tmpShuffledQuestions)
                {
                    _studyQuestions.Add(new StudyQuestion(shuffledQuestion.Id, shuffledQuestion.English, shuffledQuestion.German, shuffledQuestion.Category));
                }
            }

            for (int i = 0; i < (studyQuestionsNumber / 12) - 1; i++)
            {
                foreach (var question in _studyQuestions.GetRange(0, 12))
                {
                    _studyQuestions.Add(new StudyQuestion(question.Id, question.English, question.German, question.Category));
                }
            }
        }

        /// <summary>
        /// Saves current status to a json file for analyzing.
        /// </summary>
        public void SaveToFile()
        {
            StudyJSON studyJson = new StudyJSON(_studyQuestions, PersonalLimit);
            String jsonData = JsonUtility.ToJson(studyJson);

            File.WriteAllText(_filePath, jsonData);
        }

        /// <summary>
        /// Returns the current question of the running study
        /// </summary>
        public string CurrentQuestionText => _language == StudyController.StudyLanguage.German ? _studyQuestions[QuestionIndex].German : _studyQuestions[QuestionIndex].English;
    }
}
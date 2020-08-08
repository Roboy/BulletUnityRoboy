using System;
using System.Collections.Generic;
using System.Linq;
using Bullet;
using Controller.Helper.Study;
using RosSharp;
using TMPro;
using UnityEngine;

namespace Controller
{
    public class StudyController : MonoBehaviour
    {
        public static string savePath;

        enum StudyQuestionsNumber
        {
            ThirtySix = 36,
            FourtyEight = 48,
            Sixty = 60,
            SeventyTwo = 72
        }

        [Space(10)] [Header("General")] [SerializeField]
        private StudyQuestionsNumber studyQuestionsNumber = StudyQuestionsNumber.Sixty;

        public enum StudyLanguage
        {
            German,
            English
        }

        [SerializeField] private StudyLanguage language = StudyLanguage.German;
        [SerializeField] private BulletObject bulletObject;
        [SerializeField] private TextMeshPro guideText;


        private LimitationController _limitationController;
        private BulletSyncController _bulletSyncController;
        private BulletRobot _bulletRobot;


        [Space(10), Header("Delay")] [SerializeField]
        private float maxDelayValue = 400;

        [Space(10)] [Header("Velocity")] [SerializeField]
        private float maxVelocityValue = 10;

        private int maxJointLimitsValue => _bulletRobot ? _bulletRobot.SyncedRobots.Count : 0;

        /// <summary>
        /// Holds currently running study. If [null], no study is running.
        /// </summary>
        private Study _currentStudy = null;

        private StudyLanguage Language => language;

        private void Start()
        {
            _limitationController = GameObject.FindGameObjectWithTag("LimitationController").GetComponent<LimitationController>();
            _bulletSyncController = GameObject.FindGameObjectWithTag("BulletSyncController").GetComponent<BulletSyncController>();
            _bulletRobot = GameObject.FindGameObjectWithTag("BulletRobotController").GetComponent<BulletRobot>();

            savePath = Application.persistentDataPath;
        }

        /**
         * Key Bindings:
         * - F1: Start Delay
         * - F2: Start Velocity
         * - F3: Start Joint Limits
         * - F4: Start Combined
         * - F5: Ask (Print) curent question
         * - F9: Stop / Cancel current study
         * - F10: Reset Object
         * - F11: Previous Question
         * - F12: Next Question
         */
        private void Update()
        {
            if (Input.GetKeyDown(KeyCode.F1))
            {
                StartStudy(Study.StudyType.Delay);
            }

            if (Input.GetKeyDown(KeyCode.F2))
            {
                StartStudy(Study.StudyType.Velocity);
            }

            if (Input.GetKeyDown(KeyCode.F3))
            {
                StartStudy(Study.StudyType.JointLimits);
            }

            if (Input.GetKeyDown(KeyCode.F4))
            {
                StartStudy(Study.StudyType.Combined);
            }

            if (Input.GetKeyDown(KeyCode.F9))
            {
                StopStudy();
            }

            if (Input.GetKeyDown(KeyCode.F10))
            {
                bulletObject.UpdatePositionAndRotation((new Vector3(0, 0, 0.434f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
            }

            if (Input.GetKeyDown(KeyCode.F11))
            {
                PreviousQuestion();
            }

            if (Input.GetKeyDown(KeyCode.F12))
            {
                NextQuestion();
            }

            if (Input.GetKeyDown(KeyCode.F5))
            {
                PrintQuestion();
            }

            #region Debug: Anwser Manually

            // For Debugging: Answer Questions
            if (Input.GetKeyDown(KeyCode.Alpha1))
            {
                AnswerQuestion(1);
            }

            if (Input.GetKeyDown(KeyCode.Alpha2))
            {
                AnswerQuestion(2);
            }

            if (Input.GetKeyDown(KeyCode.Alpha3))
            {
                AnswerQuestion(3);
            }

            if (Input.GetKeyDown(KeyCode.Alpha4))
            {
                AnswerQuestion(4);
            }

            if (Input.GetKeyDown(KeyCode.Alpha5))
            {
                AnswerQuestion(5);
            }

            if (Input.GetKeyDown(KeyCode.Alpha6))
            {
                AnswerQuestion(6);
            }

            if (Input.GetKeyDown(KeyCode.Alpha7))
            {
                AnswerQuestion(7);
            }

            #endregion
        }

        #region Study Managment (Start / Stop)

        /// <summary>
        /// Starts a new study by instantiating a new study object and thus collecting data.
        /// </summary>
        /// <param name="studyType">the type of the study to be started</param>
        private void StartStudy(Study.StudyType studyType)
        {
            Debug.Log("[Study] Start " + studyType.ToString());
            if (_currentStudy != null && _currentStudy.Type != Study.StudyType.None)
            {
                StopStudy();
            }

            _currentStudy = new Study(studyType, Language, (int) studyQuestionsNumber, _limitationController);
            _currentStudy.SaveToFile();

            UpdateGuideText();
            PrintQuestion();
        }

        /// <summary>
        /// Stops a currently running study (if any)
        /// </summary>
        private void StopStudy()
        {
            if (_currentStudy == null) return;

            Debug.Log("[Study] Stop " + _currentStudy.Type.ToString());

            _currentStudy.SaveToFile();
            _currentStudy = null;
        }

        #endregion

        #region Study Managment (Answering)

        /// <summary>
        /// Jumps to the next question in the queue and updates limitations accordingly.
        /// </summary>
        /// <exception cref="ArgumentOutOfRangeException"></exception>
        private void NextQuestion()
        {
            if (_currentStudy == null || _currentStudy.Type == Study.StudyType.None) return;

            if (_currentStudy.QuestionIndex + 1 >= _currentStudy.StudyQuestions.Count)
            {
                StopStudy();
                return;
            }

            // Go to next question by increasing current index
            _currentStudy.QuestionIndex++;

            // Update limitations
            switch (_currentStudy.Type)
            {
                case Study.StudyType.None:
                    break;
                case Study.StudyType.Delay:
                    float delayValue = (maxDelayValue / (int) studyQuestionsNumber) * _currentStudy.QuestionIndex;
                    _limitationController.TrackingDelay = delayValue;
                    break;
                case Study.StudyType.Velocity:
                    float velocityValue = maxVelocityValue - ((maxVelocityValue / (int) studyQuestionsNumber) * _currentStudy.QuestionIndex);
                    _limitationController.MaxVelocity = velocityValue;
                    _limitationController.UpdateVelocity = true;
                    break;
                case Study.StudyType.JointLimits:
                    int robotIndex = (int) Math.Floor(((_currentStudy.QuestionIndex * 1.0f) / (int) studyQuestionsNumber) * maxJointLimitsValue);
                    if (_bulletRobot.SyncedRobots[robotIndex].B3RobotId != _bulletRobot.ActiveRobot.B3RobotId)
                    {
                        _limitationController.SwitchRobot = true;
                    }

                    break;
                case Study.StudyType.Combined:
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }

            UpdateGuideText();
            PrintQuestion();
        }

        /// <summary>
        /// Jumps to the previous question (without touching the limitations)
        /// </summary>
        private void PreviousQuestion()
        {
            if (_currentStudy == null || _currentStudy.Type == Study.StudyType.None) return;

            // Go to previous question by increasing current index
            _currentStudy.QuestionIndex++;

            UpdateGuideText();
            PrintQuestion();
        }

        /// <summary>
        /// Prints the current question (for debugging)
        /// </summary>
        private void PrintQuestion()
        {
            Debug.Log("[Study] Question: " + _currentStudy.CurrentQuestionText);

            if (CalculateEmbodiment() < 0)
            {
                Debug.Log("[Study] Embodiment: Not enough data.");
            }
            else
            {
                Debug.Log("[Study] Embodiment: " + CalculateEmbodiment());
            }
        }

        /// <summary>
        /// Updates the guide in the scene.
        /// </summary>
        private void UpdateGuideText()
        {
            guideText.text = _currentStudy.CurrentQuestionText;
        }

        /// <summary>
        /// Answers the current questions (likert scale, 1-7) and updates json file.
        /// </summary>
        /// <param name="answer"></param>
        public void AnswerQuestion(int answer)
        {
            if (_currentStudy == null || _currentStudy.Type == Study.StudyType.None) return;

            _currentStudy.StudyQuestions[_currentStudy.QuestionIndex].Answer = new StudyAnswer(answer, DateTime.Now.ToLongTimeString());
            _currentStudy.SaveToFile();
            
            // Reset grabable object
            bulletObject.UpdatePositionAndRotation((new Vector3(0, 0, 0.434f)).Unity2Ros(), Quaternion.identity.Unity2Ros());

            Debug.Log("[Study] Answer: " + answer);

            NextQuestion();
        }

        #endregion

        #region Embodiment Level Calculation

        /// <summary>
        /// Calculates full level of embodiment based on the last 12 measurements.
        /// </summary>
        /// <returns>embodiment level</returns>
        private float CalculateEmbodiment()
        {
            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12) return -1;

            return (CalculateBodyOwnership() / 5.0f) + (CalculateAgency() / 4.0f) + (CalculateLocation() / 3.0f);
        }

        /// <summary>
        /// Calculates body ownership based on the last 12 measurements.
        /// </summary>
        /// <returns>body ownership level</returns>
        private int CalculateBodyOwnership()
        {
            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12) return -1;

            List<StudyQuestion> relevantQuestions = _currentStudy.StudyQuestions.GetRange(_currentStudy.QuestionIndex - 12, 12);

            int q1 = relevantQuestions.Find((question => question.Id == "Q1")).Answer.Answer;
            int q2 = relevantQuestions.Find((question => question.Id == "Q2")).Answer.Answer;
            int q3 = relevantQuestions.Find((question => question.Id == "Q3")).Answer.Answer;
            int q4 = relevantQuestions.Find((question => question.Id == "Q4")).Answer.Answer;
            int q5 = relevantQuestions.Find((question => question.Id == "Q5")).Answer.Answer;

            return (q1 - q2) - q3 + (q4 - q5);
        }

        /// <summary>
        /// Calculates agency based on the last 12 measurements.
        /// </summary>
        /// <returns>agency level</returns>
        private int CalculateAgency()
        {
            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12) return -1;

            List<StudyQuestion> relevantQuestions = _currentStudy.StudyQuestions.GetRange(_currentStudy.QuestionIndex - 12, 12);

            int q6 = relevantQuestions.Find((question => question.Id == "Q6")).Answer.Answer;
            int q7 = relevantQuestions.Find((question => question.Id == "Q7")).Answer.Answer;
            int q8 = relevantQuestions.Find((question => question.Id == "Q8")).Answer.Answer;
            int q9 = relevantQuestions.Find((question => question.Id == "Q9")).Answer.Answer;

            return q6 + q7 + q8 - q9;
        }

        /// <summary>
        /// Calculates self-location level based on the last 12 measurements.
        /// </summary>
        /// <returns>self-location level</returns>
        private int CalculateLocation()
        {
            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12) return -1;

            List<StudyQuestion> relevantQuestions = _currentStudy.StudyQuestions.GetRange(_currentStudy.QuestionIndex - 12, 12);

            int q14 = relevantQuestions.Find((question => question.Id == "Q14")).Answer.Answer;
            int q15 = relevantQuestions.Find((question => question.Id == "Q15")).Answer.Answer;
            int q16 = relevantQuestions.Find((question => question.Id == "Q16")).Answer.Answer;

            return q14 - q15 + q16;
        }

        #endregion
    }
}
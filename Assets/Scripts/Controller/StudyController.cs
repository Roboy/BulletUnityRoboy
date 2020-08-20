using System;
using System.Collections.Generic;
using System.Linq;
using Bullet;
using Controller.Helper.Study;
using RosSharp;
using TMPro;
using UnityEngine;
using Valve.VR.InteractionSystem;

namespace Controller
{
    public class StudyController : MonoBehaviour
    {
        public static string savePath;

        enum StudyQuestionsNumber
        {
            TwentyFour = 24,
            ThirtySix = 36,
            FourtyEight = 48,
            Sixty = 60,
            SeventyTwo = 72
        }

        [Space(10)] [Header("General")] [SerializeField]
        private StudyQuestionsNumber studyQuestionsNumber = StudyQuestionsNumber.FourtyEight;

        public enum StudyLanguage
        {
            German,
            English
        }

        [SerializeField] private StudyLanguage language = StudyLanguage.German;
        [SerializeField] private BulletObject bulletObject;
        [SerializeField] private TextMeshPro[] guideTexts;


        private LimitationController _limitationController;
        private BulletRobot _bulletRobot;


        [Space(10), Header("Delay")] [SerializeField]
        private float maxDelayValue = 300;

        [Space(10)] [Header("Velocity")] [SerializeField]
        private float maxVelocityValue = 2.0f;

        [SerializeField] private float minVelocityValue = 0.4f;

        private int maxJointLimitsValue => _bulletRobot ? _bulletRobot.SyncedRobots.Count : 0;

        /// <summary>
        /// Holds currently running study. If [null], no study is running.
        /// </summary>
        private Study _currentStudy = null;

        /// <summary>
        /// The grabable object has three possible spawn positions.
        /// </summary>
        private int _objectPosition = 0;

        private List<StudyDropzone> _studyDropzones = new List<StudyDropzone>();

        private StudyLanguage Language => language;

        private void Start()
        {
            _limitationController = GameObject.FindGameObjectWithTag("LimitationController").GetComponent<LimitationController>();
            _bulletRobot = GameObject.FindGameObjectWithTag("BulletRobotController").GetComponent<BulletRobot>();

            foreach (var dropzoneObject in GameObject.FindGameObjectsWithTag("StudyDropzone"))
            {
                _studyDropzones.Add(dropzoneObject.GetComponentInChildren<StudyDropzone>());
            }

            _studyDropzones.ForEach((dropzone => dropzone.SetText(Language)));

            savePath = Application.persistentDataPath;

            // StudyUI is hidden by default
            ToggleStudyInterfaceVisibility(false);
            SetGuideText("");
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

            if (Input.GetKeyDown(KeyCode.F5))
            {
                StartStudy(Study.StudyType.AllIn);
            }

            if (Input.GetKeyDown(KeyCode.F9))
            {
                StopStudy();
            }

            if (Input.GetKeyDown(KeyCode.F10))
            {
                switch (_objectPosition % 3)
                {
                    case 0:
                        bulletObject.UpdatePositionAndRotation((new Vector3(-0.0634f, 0, 0.4564f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
                        break;
                    case 1:
                        bulletObject.UpdatePositionAndRotation((new Vector3(0.0995f, 0, 0.4524f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
                        break;
                    case 2:
                        bulletObject.UpdatePositionAndRotation((new Vector3(-0.225f, 0, 0.4044f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
                        break;
                }
            }

            if (Input.GetKeyDown(KeyCode.F11))
            {
                PreviousQuestion();
            }

            if (Input.GetKeyDown(KeyCode.F12))
            {
                NextQuestion();
            }

            #region Debug: Anwser Manually

            
            // For Debugging: Answer Questions
            if (Input.GetKeyDown(KeyCode.Keypad1))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.STRONGLY_DISAGREE);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.STRONGLY_DISAGREE);
            }

            if (Input.GetKeyDown(KeyCode.Alpha1))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.STRONGLY_DISAGREE);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.STRONGLY_DISAGREE);
            }

            if (Input.GetKeyDown(KeyCode.Keypad2))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.DISAGREE);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.DISAGREE);
            }
            
            if (Input.GetKeyDown(KeyCode.Alpha2))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.DISAGREE);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.DISAGREE);
            }

            if (Input.GetKeyDown(KeyCode.Keypad3))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_DISAGREE);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_DISAGREE);
            }
            
            if (Input.GetKeyDown(KeyCode.Alpha3))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_DISAGREE);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_DISAGREE);
            }

            if (Input.GetKeyDown(KeyCode.Keypad4))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.NEUTRAL);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.NEUTRAL);
            }

            if (Input.GetKeyDown(KeyCode.Alpha4))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.NEUTRAL);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.NEUTRAL);
            }
            
            if (Input.GetKeyDown(KeyCode.Keypad5))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_AGREE);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_AGREE);
            }
            
            if (Input.GetKeyDown(KeyCode.Alpha5))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_AGREE);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.SOMEWHAT_AGREE);
            }

            if (Input.GetKeyDown(KeyCode.Keypad6))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.AGREE);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.AGREE);
            }
            
            if (Input.GetKeyDown(KeyCode.Alpha6))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.AGREE);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.AGREE);
            }

            if (Input.GetKeyDown(KeyCode.Keypad7))
            {
                Debug.Log("[Study] Answer Question with " + (int) StudyDropzone.StudyAnswerOptionType.STRONGLY_AGREE);
                AnswerQuestion((int) StudyDropzone.StudyAnswerOptionType.STRONGLY_AGREE);
            }

            if (Input.GetKeyDown(KeyCode.Alpha7))
            {
                Debug.Log("[Study] Modifying Last Answer to " + (int) StudyDropzone.StudyAnswerOptionType.STRONGLY_AGREE);
                ModifyAnswer((int) StudyDropzone.StudyAnswerOptionType.STRONGLY_AGREE);
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
            ToggleStudyInterfaceVisibility(true);

            PrintQuestion();

            Debug.Log("[Study] File is at " + _currentStudy.FilePath);
        }

        /// <summary>
        /// Stops a currently running study (if any)
        /// </summary>
        private void StopStudy()
        {
            if (_currentStudy == null) return;

            Debug.Log("[Study] Stop " + _currentStudy.Type.ToString());

            ToggleStudyInterfaceVisibility(false);
            SetGuideText("");

            CalculateLimit();

            _currentStudy.SaveToFile();
            _currentStudy = null;

            UpdateDelay();
            UpdateVelocity();
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
                case Study.StudyType.AllIn:
                    break;
                case Study.StudyType.Delay:
                    UpdateDelay();
                    break;
                case Study.StudyType.Velocity:
                    UpdateVelocity();
                    break;
                case Study.StudyType.JointLimits:
                    UpdateJointLimits();
                    break;
                case Study.StudyType.Combined:
                    UpdateDelay();
                    UpdateVelocity();
                    UpdateJointLimits();
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
            if (_currentStudy == null || _currentStudy.Type == Study.StudyType.None || _currentStudy.QuestionIndex == 0) return;

            // Go to previous question by increasing current index
            _currentStudy.QuestionIndex--;

            UpdateGuideText();
            PrintQuestion();
        }

        /// <summary>
        /// Prints the current question (for debugging)
        /// </summary>
        private void PrintQuestion()
        {
            //Debug.Log("[Study] Question: " + _currentStudy.CurrentQuestionText);

            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12)
            {
                Debug.Log("[Study] Embodiment: Not enough data. (" + (_currentStudy.QuestionIndex + 1) + "/" + _currentStudy.StudyQuestions.Count + ")");
            }
            else
            {
                Debug.Log("[Study] Embodiment: " + CalculateEmbodiment() + " (" + (_currentStudy.QuestionIndex + 1) + "/" + _currentStudy.StudyQuestions.Count + ")");
            }
        }

        /// <summary>
        /// Updates the guide in the scene.
        /// </summary>
        private void UpdateGuideText()
        {
            SetGuideText(_currentStudy.CurrentQuestionText);
        }

        /// <summary>
        /// Answers the current questions (likert scale, 1-7) and updates json file.
        /// </summary>
        /// <param name="answer"></param>
        public void AnswerQuestion(int answer)
        {
            if (_currentStudy == null || _currentStudy.Type == Study.StudyType.None) return;

            _currentStudy.StudyQuestions[_currentStudy.QuestionIndex].Answer =
                new StudyAnswer(answer, CalculateEmbodiment(), CalculateBodyOwnership(), CalculateAgency(), CalculateLocation(), DateTime.Now.ToLongTimeString());
            _currentStudy.StudyQuestions[_currentStudy.QuestionIndex].State = new StudyState(_limitationController.MaxVelocity, _limitationController.TrackingDelay, _bulletRobot.ActiveRobot.B3RobotId);
            _currentStudy.SaveToFile();


            // Reset grabable object
            switch (_objectPosition % 3)
            {
                case 0:
                    bulletObject.UpdatePositionAndRotation((new Vector3(-0.0634f, 0, 0.4564f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
                    break;
                case 1:
                    bulletObject.UpdatePositionAndRotation((new Vector3(0.0995f, 0, 0.4524f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
                    break;
                case 2:
                    bulletObject.UpdatePositionAndRotation((new Vector3(-0.225f, 0, 0.4044f)).Unity2Ros(), Quaternion.identity.Unity2Ros());
                    break;
            }

            _objectPosition++;

            //Debug.Log("[Study] Answer: " + answer);

            NextQuestion();
        }

        /// <summary>
        /// Modifies a previous answer if the user missed his/her target
        /// </summary>
        public void ModifyAnswer(int answer)
        {
            if (_currentStudy == null || _currentStudy.Type == Study.StudyType.None || _currentStudy.QuestionIndex == 0)
            {
                return;
            }

            _currentStudy.QuestionIndex--;

            _currentStudy.StudyQuestions[_currentStudy.QuestionIndex].Answer =
                new StudyAnswer(answer, CalculateEmbodiment(), CalculateBodyOwnership(), CalculateAgency(), CalculateLocation(), DateTime.Now.ToLongTimeString());
            _currentStudy.SaveToFile();

            _currentStudy.QuestionIndex++;
        }

        #endregion

        #region Embodiment Level Calculation

        /// <summary>
        /// Calculates full level of embodiment based on the last 12 measurements.
        /// </summary>
        /// <returns>embodiment level</returns>
        private float CalculateEmbodiment()
        {
            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12) return -100.0f;

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

        /// <summary>
        /// Calculates, at which point (i.e. limitation value) in the current study the embodiment level drops below a critical threshold.
        /// This assumes, that embodiment is lost after this critical threshold.
        /// It's used to check later, whether embodiment one step previously (where embodiment levels where still higher) can be achieved, if a user is exposed to these values directly.
        /// </summary>
        private void CalculateLimit()
        {
            float maxEmbodiment = _currentStudy.StudyQuestions.GetRange(12, _currentStudy.StudyQuestions.Count - 12).Max((question => question.Answer.Embodiment)); // 100%
            float minEmbodiment = _currentStudy.StudyQuestions.GetRange(12, _currentStudy.StudyQuestions.Count - 12).Min((question => question.Answer.Embodiment)); // 0%

            float targetEmbodiment = minEmbodiment + (((maxEmbodiment - minEmbodiment) / 100.0f) * 60.0f); // 60%

            StudyQuestion targetQuestion = _currentStudy.StudyQuestions.GetRange(12, _currentStudy.StudyQuestions.Count - 12).FindLast((question => question.Answer.Embodiment >= targetEmbodiment));

            Debug.Log("[Study] Max: " + maxEmbodiment + " | Min: " + minEmbodiment + " | Target: " + targetEmbodiment);
            Debug.Log("[Study] " + JsonUtility.ToJson(targetQuestion));

            switch (_currentStudy.Type)
            {
                case Study.StudyType.None:
                case Study.StudyType.Combined:
                case Study.StudyType.AllIn:
                    _currentStudy.PersonalLimit.Delay = targetQuestion.State.Delay;
                    _currentStudy.PersonalLimit.Velocity = targetQuestion.State.Velocity;
                    _currentStudy.PersonalLimit.RobotId = targetQuestion.State.RobotId;

                    Debug.Log("[Study] Delay Limit: " + _currentStudy.PersonalLimit.Delay);
                    Debug.Log("[Study] Velocity Limit: " + _currentStudy.PersonalLimit.Velocity);
                    Debug.Log("[Study] Robot Limitation Limit: " + _currentStudy.PersonalLimit.RobotId);
                    break;
                case Study.StudyType.Delay:
                    _currentStudy.PersonalLimit.Delay = targetQuestion.State.Delay;
                    Debug.Log("[Study] Delay Limit: " + _currentStudy.PersonalLimit.Delay);
                    break;
                case Study.StudyType.Velocity:
                    _currentStudy.PersonalLimit.Velocity = targetQuestion.State.Velocity;
                    Debug.Log("[Study] Velocity Limit: " + _currentStudy.PersonalLimit.Velocity);
                    break;
                case Study.StudyType.JointLimits:
                    _currentStudy.PersonalLimit.RobotId = targetQuestion.State.RobotId;
                    Debug.Log("[Study] Robot Limitation Limit: " + _currentStudy.PersonalLimit.RobotId);
                    break;
                default:
                    throw new ArgumentOutOfRangeException();
            }
        }

        #endregion

        /// <summary>
        /// Hides or shows the UI for the study, namely the guide texts and dropzones.
        /// </summary>
        /// <param name="visibility">true for visible, false for invisible</param>
        private void ToggleStudyInterfaceVisibility(bool visibility)
        {
            GameObject[] studyUI = GameObject.FindGameObjectsWithTag("StudyUI");
            foreach (GameObject gameObject in studyUI)
            {
                if (gameObject.GetComponent<Renderer>() != null)
                {
                    gameObject.GetComponent<Renderer>().enabled = visibility;
                }
            }
        }

        /// <summary>
        /// Updates all guide texts existing in the scene.
        /// </summary>
        /// <param name="text">the new text</param>
        private void SetGuideText(string text)
        {
            foreach (TextMeshPro guideText in guideTexts)
            {
                guideText.text = text;
            }
        }

        /// <summary>
        /// Updates the velocity limitation to the correct value depending on [_currentStudy.QuestionIndex]
        /// </summary>
        private void UpdateVelocity()
        {
            if (_currentStudy == null)
            {
                _limitationController.MaxVelocity = maxVelocityValue;
                _limitationController.UpdateVelocity = true;
                return;
            }

            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12)
            {
                return;
            }

            float velocityValue = maxVelocityValue - (((maxVelocityValue - minVelocityValue) / ((int) studyQuestionsNumber - 12)) * (_currentStudy.QuestionIndex - 11));
            _limitationController.MaxVelocity = velocityValue;
            _limitationController.UpdateVelocity = true;
        }

        /// <summary>
        /// Updates the delay limitation to the correct value depending on [_currentStudy.QuestionIndex]
        /// </summary>
        private void UpdateDelay()
        {
            if (_currentStudy == null)
            {
                _limitationController.TrackingDelay = 0;
                return;
            }

            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12)
            {
                return;
            }

            float delayValue = (maxDelayValue / ((int) studyQuestionsNumber - 12)) * (_currentStudy.QuestionIndex - 11);
            _limitationController.TrackingDelay = delayValue;
        }

        /// <summary>
        /// Updates the joint limitation to the correct value depending on [_currentStudy.QuestionIndex]
        /// </summary>
        private void UpdateJointLimits()
        {
            if (_currentStudy.StudyQuestions.Count((question => question.Answer != null)) < 12)
            {
                return;
            }

            int robotIndex = (int) Math.Floor((((_currentStudy.QuestionIndex - 12) * 1.0f) / ((int) studyQuestionsNumber - 12)) * maxJointLimitsValue);
            if (_bulletRobot.SyncedRobots[robotIndex].B3RobotId != _bulletRobot.ActiveRobot.B3RobotId)
            {
                if (!_limitationController.SwitchInProgress)
                {
                    _limitationController.SwitchRobot = true;
                }
            }
        }
    }
}
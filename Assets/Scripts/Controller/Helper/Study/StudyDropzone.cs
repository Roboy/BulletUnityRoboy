using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

namespace Controller.Helper.Study
{
    public class StudyDropzone : MonoBehaviour
    {
        public enum StudyAnswerOptionType
        {
            STRONGLY_DISAGREE = -3,
            DISAGREE = -2,
            SOMEWHAT_DISAGREE = -1,
            NEUTRAL = 0,
            SOMEWHAT_AGREE = 1,
            AGREE = 2,
            STRONGLY_AGREE = 3
        }

        [SerializeField] private StudyAnswerOptionType answerOptionType;
        [SerializeField] private TextMeshPro informationText;

        private StudyController _studyController;

        private readonly List<StudyAnswerOption> _allStudyAnswerOptions = new List<StudyAnswerOption>()
        {
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.STRONGLY_DISAGREE, "Strongly Disagree", "Stimme überhaupt\nnicht zu"),
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.DISAGREE, "Disagree", "Stimme nicht zu"),
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.SOMEWHAT_DISAGREE, "Somewhat Disagree", "Stimme teilweise\nnicht zu"),
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.NEUTRAL, "Neutral", "Neutral"),
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.SOMEWHAT_AGREE, "Somewhat Agree", "Stimme teilweise zu"),
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.AGREE, "Agree", "Stimme zu"),
            new StudyAnswerOption(StudyDropzone.StudyAnswerOptionType.STRONGLY_AGREE, "Strongly Agree", "Stimme völlig zu")
        };
        
        class RendererWrapper
        {
            public Renderer Renderer { get; set; }
            public Material Material { get; set; }

            public RendererWrapper(Renderer renderer, Material material)
            {
                Renderer = renderer;
                Material = material;
            }
        }
        private List<RendererWrapper> _renderers = new List<RendererWrapper>();

        private void Start()
        {
            _studyController = GameObject.FindGameObjectWithTag("StudyController").GetComponent<StudyController>();
            //_renderers = transform.parent.gameObject.GetComponentsInChildren<Renderer>();
            foreach (var renderer in transform.parent.gameObject.GetComponentsInChildren<Renderer>())
            {
                _renderers.Add(new RendererWrapper(renderer, renderer.material));
            }
        }
        
        private void OnTriggerEnter(Collider other)
        {
            _studyController.AnswerQuestion((int) answerOptionType);
            StartCoroutine(FlashOnTrigger());
        }

        IEnumerator FlashOnTrigger()
        {
            foreach (var rendererWrapper in _renderers)
            {
                rendererWrapper.Renderer.material = null;
                rendererWrapper.Renderer.material.color = Color.green;
            }

            yield return new WaitForSeconds(0.1f);

            foreach (var rendererWrapper in _renderers)
            {
                rendererWrapper.Renderer.material = rendererWrapper.Material;
            }
        }

        public void SetText(StudyController.StudyLanguage studyLanguage)
        {
            StudyAnswerOption studyAnswerOption = _allStudyAnswerOptions.Find((option => option.StudyAnswerOptionType == answerOptionType));
            if (studyLanguage == StudyController.StudyLanguage.English)
            {
                this.informationText.text = "" + (int) studyAnswerOption.StudyAnswerOptionType + "\n" + studyAnswerOption.English;
            }
            else
            {
                this.informationText.text = "" + (int) studyAnswerOption.StudyAnswerOptionType + "\n" + studyAnswerOption.German;
            }
        }
    }
}
using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Controller.Helper.Study
{
    public class StudyDropzone : MonoBehaviour
    {
        public enum StudyAnswerOption
        {
            One = 1,
            Two = 2,
            Three = 3,
            Four = 4,
            Five = 5,
            Six = 6,
            Seven = 7
        }

        [SerializeField] private StudyAnswerOption answerOption;

        private StudyController _studyController;

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
            Debug.Log("D1");
            _studyController.AnswerQuestion((int) answerOption);
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
    }
}
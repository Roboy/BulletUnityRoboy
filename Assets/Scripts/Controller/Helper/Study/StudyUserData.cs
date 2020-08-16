using System;
using UnityEngine;
using Utils;

namespace Controller.Helper.Study
{
    /// <summary>
    /// Holds personal information for identification of the user currently performing the study.
    /// </summary>
    public class StudyUserData: Singleton<StudyUserData>
    {
        [SerializeField] private string forename;
        [SerializeField] private string surname;
        [SerializeField] private int age;
        [SerializeField] private string uuid;

        public string Forename => forename;

        public string Surname => surname;

        public int Age => age;

        public string Uuid => uuid;
    }
}
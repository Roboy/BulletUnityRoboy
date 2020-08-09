using System;
using Utils;

namespace Controller.Helper.Study
{
    public class StudySession: Singleton<StudySession>
    {
        public Guid Guid { get; private set; }

        private void Start()
        {
            Guid = Guid.NewGuid();
        }
    }
}
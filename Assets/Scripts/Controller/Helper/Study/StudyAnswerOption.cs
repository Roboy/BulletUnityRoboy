namespace Controller.Helper.Study
{
    public class StudyAnswerOption
    {
        private StudyDropzone.StudyAnswerOptionType _studyAnswerOptionType;
        private string _german;
        private string _english;
        private int _value;

        public StudyAnswerOption(StudyDropzone.StudyAnswerOptionType studyAnswerOptionType, string english, string german)
        {
            _studyAnswerOptionType = studyAnswerOptionType;
            _german = german;
            _english = english;
        }

        public StudyDropzone.StudyAnswerOptionType StudyAnswerOptionType => _studyAnswerOptionType;

        public string German => _german;

        public string English => _english;
    }
}
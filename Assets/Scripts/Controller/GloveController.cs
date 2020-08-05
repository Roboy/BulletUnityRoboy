using SenseGloveCs;
using TreeEditor;
using UnityEngine;
using Utils;

namespace Controller
{
    public class GloveController : MonoBehaviour
    {
        [SerializeField] [Header("SenseGlove (Right Hand)")]
        private SenseGlove_Object trackedGlove;

        public class GloveContactStatusInformation
        {
            private readonly CircularQueue<bool> _thumbQueue = new CircularQueue<bool>(25);
            private readonly CircularQueue<bool> _indexQueue = new CircularQueue<bool>(25);
            private readonly CircularQueue<bool> _middleQueue = new CircularQueue<bool>(25);
            private readonly CircularQueue<bool> _ringQueue = new CircularQueue<bool>(25);
            private readonly CircularQueue<bool> _pinkyQueue = new CircularQueue<bool>(25);

            public bool Thumb => _thumbQueue.CountVar(true) > 10;

            public bool Index => _indexQueue.CountVar(true) > 10;

            public bool Middle => _middleQueue.CountVar(true) > 10;

            public bool Ring => _ringQueue.CountVar(true) > 10;

            public bool Pinky => _pinkyQueue.CountVar(true) > 10;

            public CircularQueue<bool> ThumbQueue => _thumbQueue;

            public CircularQueue<bool> IndexQueue => _indexQueue;

            public CircularQueue<bool> MiddleQueue => _middleQueue;

            public CircularQueue<bool> RingQueue => _ringQueue;

            public CircularQueue<bool> PinkyQueue => _pinkyQueue;
        }

        private readonly GloveContactStatusInformation _gloveContactStatus = new GloveContactStatusInformation();

        public GloveContactStatusInformation GloveContactStatus => _gloveContactStatus;

        private void Update()
        {
            trackedGlove.SendBrakeCmd(_gloveContactStatus.Thumb ? 100 : 0, _gloveContactStatus.Index ? 100 : 0, _gloveContactStatus.Middle ? 100 : 0, _gloveContactStatus.Ring ? 100 : 0,
                _gloveContactStatus.Pinky ? 100 : 0);
        }
    }
}
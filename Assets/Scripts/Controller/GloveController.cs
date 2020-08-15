using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Controller
{
    public class GloveFingerContactInformation
    {
        private bool _hasCollision;
        private float _atTime;

        public bool HasCollision => _hasCollision;

        public float AtTime => _atTime;

        public GloveFingerContactInformation(bool hasCollision, float atTime)
        {
            _hasCollision = hasCollision;
            _atTime = atTime;
        }
    }

    public class GloveController : MonoBehaviour
    {
        [SerializeField] [Header("SenseGlove (Right Hand)")]
        private SenseGlove_Object trackedGlove;

        public class GloveContactStatusInformation
        {
            private BulletBridge _bulletBridge;

            public int MaxObjectsPerQueue { get; } = 250;
            public int RelevantObjectsPerLookup { get; } = 25;

            public GloveContactStatusInformation(BulletBridge bulletBridge)
            {
                _bulletBridge = bulletBridge;
            }

            private readonly List<GloveFingerContactInformation> _thumbQueue = new List<GloveFingerContactInformation>();
            private readonly List<GloveFingerContactInformation> _indexQueue = new List<GloveFingerContactInformation>();
            private readonly List<GloveFingerContactInformation> _middleQueue = new List<GloveFingerContactInformation>();
            private readonly List<GloveFingerContactInformation> _ringQueue = new List<GloveFingerContactInformation>();
            private readonly List<GloveFingerContactInformation> _pinkyQueue = new List<GloveFingerContactInformation>();


            public bool Thumb
            {
                get
                {
                    lock (_thumbQueue)
                    {
                        bool shouldBrake = _thumbQueue.Where((information => information.AtTime < _bulletBridge.CurrentTime)).Reverse().Take(RelevantObjectsPerLookup).Count((information => information.HasCollision)) >
                                           10;

                        return shouldBrake;
                    }
                }
            }

            public bool Index
            {
                get
                {
                    lock (_indexQueue)
                    {
                        bool shouldBrake = _indexQueue.Where((information => information.AtTime < _bulletBridge.CurrentTime)).Reverse().Take(RelevantObjectsPerLookup).Count((information => information.HasCollision)) >
                                           10;

                        return shouldBrake;
                    }
                }
            }

            public bool Middle
            {
                get
                {
                    lock (_middleQueue)
                    {
                        bool shouldBrake = _middleQueue.Where((information => information.AtTime < _bulletBridge.CurrentTime)).Reverse().Take(RelevantObjectsPerLookup).Count((information => information.HasCollision)) >
                                           10;

                        return shouldBrake;
                    }
                }
            }

            public bool Ring
            {
                get
                {
                    lock (_ringQueue)
                    {
                        bool shouldBrake = _ringQueue.Where((information => information.AtTime < _bulletBridge.CurrentTime)).Reverse().Take(RelevantObjectsPerLookup).Count((information => information.HasCollision)) > 10;

                        return shouldBrake;
                    }
                }
            }

            public bool Pinky
            {
                get
                {
                    lock (_pinkyQueue)
                    {
                        bool shouldBrake = _pinkyQueue.Where((information => information.AtTime < _bulletBridge.CurrentTime)).Reverse().Take(RelevantObjectsPerLookup).Count((information => information.HasCollision)) >
                                           10;

                        return shouldBrake;
                    }
                }
            }

            public List<GloveFingerContactInformation> ThumbQueue => _thumbQueue;

            public List<GloveFingerContactInformation> IndexQueue => _indexQueue;

            public List<GloveFingerContactInformation> MiddleQueue => _middleQueue;

            public List<GloveFingerContactInformation> RingQueue => _ringQueue;

            public List<GloveFingerContactInformation> PinkyQueue => _pinkyQueue;
        }

        private BulletBridge _bulletBridge;
        private LimitationController _limitationController;
        private GloveContactStatusInformation _gloveContactStatus;

        public GloveContactStatusInformation GloveContactStatus => _gloveContactStatus;

        private void Start()
        {
            _bulletBridge = GameObject.FindGameObjectWithTag("BulletConnectionController").GetComponent<BulletBridge>();

            _gloveContactStatus = new GloveContactStatusInformation(_bulletBridge);
        }

        private void Update()
        {
            trackedGlove.SendBrakeCmd(_gloveContactStatus.Thumb ? 100 : 0, _gloveContactStatus.Index ? 100 : 0, _gloveContactStatus.Middle ? 100 : 0, _gloveContactStatus.Ring ? 100 : 0,
                _gloveContactStatus.Pinky ? 100 : 0);
        }
    }
}
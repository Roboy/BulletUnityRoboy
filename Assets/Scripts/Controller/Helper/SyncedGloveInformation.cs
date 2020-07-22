using System;
using System.Collections.Generic;
using UnityEngine;

namespace Controller.Helper
{
    public class SyncedGloveJoint
    {
        private int _jointIndex;
        private String _jointName;
        private double _targetJointPos;
        private List<Transform> _jointTransform;

        public SyncedGloveJoint(int jointIndex, string jointName, List<Transform> jointTransform)
        {
            _jointIndex = jointIndex;
            _jointName = jointName;
            _jointTransform = jointTransform;
        }

        public int JointIndex
        {
            get => _jointIndex;
            set => _jointIndex = value;
        }

        public string JointName
        {
            get => _jointName;
            set => _jointName = value;
        }

        public double TargetJointPos
        {
            get => _targetJointPos;
            set => _targetJointPos = value;
        }

        public List<Transform> JointTransform
        {
            get => _jointTransform;
            set => _jointTransform = value;
        }
    }

    public class SyncedGloveInformation
    {
        private List<List<Transform>> _jointTransforms;
        private List<List<Quaternion>> _jointCorrections;
        private List<List<SyncedGloveJoint>> _handJointInformation;

        public List<List<Transform>> JointTransforms => _jointTransforms;

        public List<List<SyncedGloveJoint>> HandJointInformation => _handJointInformation;

        public List<List<Quaternion>> JointCorrections => _jointCorrections;

        public SyncedGloveInformation(GameObject senseGlove, String handPrefix, List<SyncedJointsInformation> syncedJointsInformations)
        {
            SenseGlove_VirtualHand _senseGloveVirtualHand = senseGlove.GetComponent<SenseGlove_VirtualHand>();
            _jointTransforms = new List<List<Transform>>()
            {
                _senseGloveVirtualHand.thumbJoints,
                _senseGloveVirtualHand.indexFingerJoints,
                _senseGloveVirtualHand.middleFingerJoints,
                _senseGloveVirtualHand.ringFingerJoints,
                _senseGloveVirtualHand.pinkyJoints
            };

            _jointCorrections = _senseGloveVirtualHand.fingerCorrection;

            _handJointInformation = new List<List<SyncedGloveJoint>>()
            {
                // thumbJoints
                new List<SyncedGloveJoint>
                {
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ5").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ5").JointName,
                        _senseGloveVirtualHand.thumbJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ4").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ4").JointName,
                        _senseGloveVirtualHand.thumbJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ3").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ3").JointName,
                        _senseGloveVirtualHand.thumbJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ2").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ2").JointName,
                        _senseGloveVirtualHand.thumbJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ1").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "THJ1").JointName,
                        _senseGloveVirtualHand.thumbJoints
                    ),
                },
                // indexFingerJoints
                new List<SyncedGloveJoint>
                {
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "FFJ3").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "FFJ3").JointName,
                        _senseGloveVirtualHand.indexFingerJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "FFJ2").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "FFJ2").JointName,
                        _senseGloveVirtualHand.indexFingerJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "FFJ1").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "FFJ1").JointName,
                        _senseGloveVirtualHand.indexFingerJoints
                    ),
                },
                // middleFingerJoints
                new List<SyncedGloveJoint>
                {
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "MFJ3").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "MFJ3").JointName,
                        _senseGloveVirtualHand.middleFingerJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "MFJ2").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "MFJ2").JointName,
                        _senseGloveVirtualHand.middleFingerJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "MFJ1").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "MFJ1").JointName,
                        _senseGloveVirtualHand.middleFingerJoints
                    ),
                },
                // ringFingerJoints
                new List<SyncedGloveJoint>
                {
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "RFJ3").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "RFJ3").JointName,
                        _senseGloveVirtualHand.ringFingerJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "RFJ2").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "RFJ2").JointName,
                        _senseGloveVirtualHand.ringFingerJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "RFJ1").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "RFJ1").JointName,
                        _senseGloveVirtualHand.ringFingerJoints
                    ),
                },
                // pinkyJoints
                new List<SyncedGloveJoint>
                {
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "LFJ3").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "LFJ3").JointName,
                        _senseGloveVirtualHand.pinkyJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "LFJ2").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "LFJ2").JointName,
                        _senseGloveVirtualHand.pinkyJoints
                    ),
                    new SyncedGloveJoint(
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "LFJ1").JointIndex,
                        syncedJointsInformations.Find((jointToSync) => jointToSync.JointName == handPrefix + "LFJ1").JointName,
                        _senseGloveVirtualHand.pinkyJoints
                    ),
                }
            };
        }
    }
}
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class BulletSenseGlove : MonoBehaviour
{

    //private List<GloveInfo> glovesInfo;
    ////private SenseGlove_VirtualHand virtualHand;
    ////private List<List<Transform>> virtualHandJointsTF;
    struct GloveInfo
    {
        public GloveInfo(GameObject glove, List<int> jointIds, List<string> jointNames)
        {
            var vHand = glove.GetComponent<SenseGlove_VirtualHand>();

            isRight = glove.name.Contains("Right");

            //vHand.senseGlove.IsRight;// glove.GetComponent<SenseGlove_Object>().IsRight;
            jointTFs = new List<List<Transform>>()
            {
                vHand.thumbJoints,
                vHand.indexFingerJoints,
                vHand.middleFingerJoints,
                vHand.ringFingerJoints,
                vHand.pinkyJoints
            };
            jointCorrections = vHand.fingerCorrection;
            var pref = "lh_";
            if (isRight) pref = "rh_";
            handJointsIds = new List<List<int>>(){
                new List<int> {
                    jointIds.ElementAt(jointNames.IndexOf(pref + "THJ5")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "THJ4")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "THJ3")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "THJ2"))
                    //b3JointIds.ElementAt(jointNames.IndexOf(pref + "THJ1"))

                },
                new List<int> {
                    jointIds.ElementAt(jointNames.IndexOf(pref + "FFJ3")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "FFJ2")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "FFJ1"))
                },
                new List<int> {
                    jointIds.ElementAt(jointNames.IndexOf(pref + "MFJ3")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "MFJ2")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "MFJ1"))
                },
                new List<int> {
                    jointIds.ElementAt(jointNames.IndexOf(pref + "RFJ3")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "RFJ2")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "RFJ1"))
                },
                new List<int> {
                    jointIds.ElementAt(jointNames.IndexOf(pref + "LFJ3")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "LFJ2")),
                    jointIds.ElementAt(jointNames.IndexOf(pref + "LFJ1"))
                }
            };
            thumbPrevSetpoints = new List<double>(new double[handJointsIds[0].Count]);
        }
        public List<List<Transform>> jointTFs;
        public List<List<Quaternion>> jointCorrections;
        public bool isRight;
        public List<List<int>> handJointsIds;
        public List<double> thumbPrevSetpoints;
    }

    // Start is called before the first frame update
    void Start()
    {
        //glovesInfo = new List<GloveInfo>();
        //foreach (var glove in senseGloves)
        //{
        //    glovesInfo.Add(new GloveInfo(glove, b3JointIds, jointNames));
        //}
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    //void trackFingerJoints(GloveInfo glove)
    //{

    //    var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);

    //    for (int i = 0; i < glove.handJointsIds.Count; i++)
    //    {
    //        List<Transform> joints = glove.jointTFs[i];// new List<Transform>();
    //        for (int j = 0; j < glove.handJointsIds[i].Count; j++)
    //        {
    //            Quaternion q;
    //            double targetJointPos;
    //            if (i == 0) // thumb is really special
    //            {
    //                switch (j)
    //                {
    //                    case 0:
    //                        q = (joints[0].rotation * glove.jointCorrections[i][0]).Unity2Ros();
    //                        targetJointPos = clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad + 0.6;
    //                        break;
    //                    case 1:
    //                        q = (joints[0].rotation * glove.jointCorrections[i][0]).Unity2Ros();
    //                        targetJointPos = clipAngle(q.eulerAngles.y) * Mathf.Deg2Rad;
    //                        break;
    //                    case 2:
    //                        q = (joints[0].rotation * glove.jointCorrections[i][0]).Unity2Ros();
    //                        targetJointPos = clipAngle(-q.eulerAngles.z) * Mathf.Deg2Rad;
    //                        break;
    //                    default:
    //                        q = (joints[j - 2].rotation * glove.jointCorrections[i][j - 2]).Unity2Ros();
    //                        targetJointPos = clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad;
    //                        break;
    //                }

    //            }
    //            else
    //            {
    //                q = (joints[j].rotation * glove.jointCorrections[i][j]).Unity2Ros();
    //                switch (j)
    //                {
    //                    case 0:
    //                        targetJointPos = clipAngle(-q.eulerAngles.z + 100) * Mathf.Deg2Rad;
    //                        break;
    //                    case 1:
    //                        targetJointPos = clipAngle(-q.eulerAngles.z + 60) * Mathf.Deg2Rad;
    //                        break;
    //                    default:
    //                        targetJointPos = clipAngle(-q.eulerAngles.z - 20) * Mathf.Deg2Rad;
    //                        break;
    //                }
    //            }
    //            setJointPosition(ref cmd, glove.handJointsIds[i][j], targetJointPos);
    //        }
    //    }
    //    NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    //}
}

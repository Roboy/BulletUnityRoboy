using RosSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using TMPro;
using UnityEngine;

public class BulletSenseGlove : MonoBehaviour
{
    public GameObject glove;
    public GameObject text;
    private GloveInfo gloveInfo;
    private IntPtr pybullet;
    private BulletBridge bb;
    public BulletRobot robot;

    public SenseGlove_Object senseGlove;
    ////private List<List<Transform>> virtualHandJointsTF;
    List<Vector3> maxTipPositions, minTipPositions, tipPositions;
    bool minCalibrated, maxCalibrated;
    List<float> steps;
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
       
        bb = GameObject.Find("BulletBridge").GetComponent<BulletBridge>();
        if (!bb.isInitialized)
        {
            Debug.LogError("no bullet bridge");
            Application.Quit();
        }
        pybullet = bb.GetPhysicsServerPtr();
        Debug.Log("Connected " + name + " to bullet server.");

        //gloveInfo = new GloveInfo(glove, robot.Getb3JointIds(), robot.GetJointNames());
        maxTipPositions = new List<Vector3>();
        minTipPositions = new List<Vector3>();
        tipPositions = new List<Vector3>();
        steps = new List<float>();
    }

    // Update is called once per frame
    void Update()
    {
        if (!maxCalibrated && Input.GetKeyDown(KeyCode.Space))
        {
            foreach (var finger in senseGlove.GloveData.handPositions)
            {
                maxTipPositions.Add(finger.AsQueryable().Last());
            }
            maxCalibrated = true;
            text.GetComponent<TextMeshPro>().text = "Close your hand and press SPACE...";
        }
        else if (!minCalibrated && Input.GetKeyDown(KeyCode.Space))
        {
            foreach (var finger in senseGlove.GloveData.handPositions)
            {
                minTipPositions.Add(finger.AsQueryable().Last());
            }
            minCalibrated = true;
            for (int i = 0; i < minTipPositions.Count; i++)
            {
                var min = minTipPositions[i];
                var max = maxTipPositions[i];
                var dist = Vector3.Distance(min, max);
                var step = dist / 800.0f;
                steps.Add(step);
            }
        }

    }

    void trackFingerJointsSim()
    {

        var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, /*robot.Getb3RobotId()*/ 0, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);

        for (int i = 0; i < gloveInfo.handJointsIds.Count; i++)
        {
            List<Transform> joints = gloveInfo.jointTFs[i];// new List<Transform>();
            for (int j = 0; j < gloveInfo.handJointsIds[i].Count; j++)
            {
                Quaternion q;
                double targetJointPos;
                if (i == 0) // thumb is really special
                {
                    switch (j)
                    {
                        case 0:
                            q = (joints[0].rotation * gloveInfo.jointCorrections[i][0]).Unity2Ros();
                            targetJointPos = BulletBridge.ClipAngle(q.eulerAngles.x) * Mathf.Deg2Rad + 0.6;
                            break;
                        case 1:
                            q = (joints[0].rotation * gloveInfo.jointCorrections[i][0]).Unity2Ros();
                            targetJointPos = BulletBridge.ClipAngle(q.eulerAngles.y) * Mathf.Deg2Rad;
                            break;
                        case 2:
                            q = (joints[0].rotation * gloveInfo.jointCorrections[i][0]).Unity2Ros();
                            targetJointPos = BulletBridge.ClipAngle(-q.eulerAngles.z) * Mathf.Deg2Rad;
                            break;
                        default:
                            q = (joints[j - 2].rotation * gloveInfo.jointCorrections[i][j - 2]).Unity2Ros();
                            targetJointPos = BulletBridge.ClipAngle(q.eulerAngles.x) * Mathf.Deg2Rad;
                            break;
                    }

                }
                else
                {
                    q = (joints[j].rotation * gloveInfo.jointCorrections[i][j]).Unity2Ros();
                    switch (j)
                    {
                        case 0:
                            targetJointPos = BulletBridge.ClipAngle(-q.eulerAngles.z + 100) * Mathf.Deg2Rad;
                            break;
                        case 1:
                            targetJointPos = BulletBridge.ClipAngle(-q.eulerAngles.z + 60) * Mathf.Deg2Rad;
                            break;
                        default:
                            targetJointPos = BulletBridge.ClipAngle(-q.eulerAngles.z - 20) * Mathf.Deg2Rad;
                            break;
                    }
                }
                bb.SetJointPosition(ref cmd, /*robot.Getb3RobotId()*/ 0, gloveInfo.handJointsIds[i][j], targetJointPos);
            }
        }
        NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    }

    void startCalibrationHW()
    {
        minCalibrated = false;
        maxCalibrated = false;
        text.SetActive(true);
        text.GetComponent<TextMeshPro>().text = "Open your hand and press SPACE...";

    }

    void updateFingerTips()
    {
        foreach (var finger in senseGlove.GloveData.handPositions)
        {
            tipPositions.Add(finger.AsQueryable().Last());
        }

    }

    public List<float> getBionicHandCmd()
    {
        var cmds = new List<float>();
        float cmd;
        for (int i = 0; i < tipPositions.Count - 1; i++)
        {
            var cur = tipPositions[i];
            var init = maxTipPositions[i];
            var dist = Vector3.Distance(cur, init);
            cmd = dist / steps[i];
            if (cmd > 700)
            {
                cmd = 700;
            }
            else if (cmd < 0)
            {
                cmd = 0;
            }
            cmds.Add(cmd);
        }
        return cmds;

    }
}
﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Urdf;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System;
using Valve.VR;
using UnityEditorInternal;
using System.Linq;
using System.Security.Principal;
using RosSharp;
using PathCreation;


[RequireComponent(typeof(UrdfRobot))]
public class BulletYumi : MonoBehaviour
{
    public Transform cameraTF;
    public bool trackIK;
    public bool followPath;
    public Transform leftHandTarget;
    public Transform rightHandTarget;
    public string urdfPath;

    public PathCreator pathCreator;
    public EndOfPathInstruction endOfPathInstruction;
    public float speed = 5;
    float distanceTravelled;

    private IntPtr pybullet;
    private BulletBridge bb;
    private List<BulletRobot.IKTarget> IKTargets;
    private float lastUpdate;

    private UrdfRobot urdfRobot;
    private List<int> b3JointIds;
    private List<string> b3JointNames;
    private List<string> jointNames; // synced with b3JointIds
    private List<int> headJointIds;
    private List<int> freeJoints;
    private List<int> wristJoints;
    private List<int> excludeFromIkJoints;
    private int b3RobotId;
    private double camYOffset;
    private Vector3 initRobotPosition;


    // Start is called before the first frame update
    void Start()
    {
        Physics.autoSimulation = false;
        bb = GameObject.Find("BulletBridge").GetComponent<BulletBridge>();
        if (!bb.isInitialized)
        {
            Debug.LogError("no bullet bridge");
            Application.Quit();
        }
        //while (!bb.isInitialized)
        //{
        //    bb.WaitForConnection();
        //}
        pybullet = bb.GetPhysicsServerPtr();
        Debug.Log("Connected " + name + " to bullet server.");

        BulletBridge.MakeKinematic(GetComponentsInChildren<Rigidbody>());
        

        resetRobotPose();
        urdfRobot = GetComponent<UrdfRobot>();
        var robotPath = Application.dataPath + urdfPath;
        b3RobotId = bb.LoadURDF(robotPath, transform.position, transform.rotation, 1);
        bb.AddGameObject(gameObject, b3RobotId);

        //setupRobotJoints();
        //if (trackIK)
        //{
        //    IKTargets = new List<BulletRobot.IKTarget>()
        //    {
        //        //new BulletRobot.IKTarget(rightHandTarget, bb.b3GetLinkId("gripper_r_base", b3RobotId), bb.GetJointKinematicChain(b3RobotId, "gripper_r_base", "yumi_link_1_r")),
        //        new BulletRobot.IKTarget(leftHandTarget, bb.b3GetLinkId("gripper_l_base", b3RobotId), bb.GetJointKinematicChain(b3RobotId, "gripper_l_base", "yumi_body"))
        //    };
        //}

        //if(followPath )
        //{
        //    if (pathCreator != null)
        //    {
        //        // Subscribed to the pathUpdated event so that we're notified if the path changes during the game
        //        pathCreator.pathUpdated += OnPathChanged;
        //    }
        //}

        lastUpdate = Time.time * 1000;

        //camYOffset = cameraTF.rotation.eulerAngles.y;

        var cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);


    }

    // Update is called once per frame
    void Update()
    {
        foreach (var link in GetComponentsInChildren<UrdfLink>())
        {
            bb.SyncLinkPoseBullet2Unity(link, b3RobotId);
        }
        //syncRobotJointStates(ref urdfRobot);
        if (Time.time * 1000 - lastUpdate > 20)
        {
            //trackHead();
            if (trackIK)
            {
                foreach (var target in IKTargets)
                    followObjectIK(target);
            }

            if (followPath)
            {
                if (pathCreator != null)
                {
                    distanceTravelled += speed * Time.deltaTime;
                    GameObject targetObj = new GameObject();
                    
                    if (pathCreator.transform != null)
                    {
                        targetObj.transform.position = pathCreator.path.GetPointAtDistance(distanceTravelled, endOfPathInstruction);
                        targetObj.transform.rotation = pathCreator.path.GetRotationAtDistance(distanceTravelled, endOfPathInstruction);
                        var t = new BulletRobot.IKTarget(targetObj.transform, bb.b3GetLinkId("gripper_l_base", b3RobotId), bb.GetJointKinematicChain(b3RobotId, "gripper_l_base", "yumi_body"));
                        followObjectIK(t);
                    }
                    
                }
            }

        }
    }

    void OnPathChanged()
    {
        distanceTravelled = pathCreator.path.GetClosestDistanceAlongPath(transform.position);
    }
    void resetRobotPose()
    {
        //var p = -cameraTF.up + cameraTF.forward * -0.1f + cameraTF.position;
        //var cam_rotation = cameraTF.rotation;
        //Quaternion q = Quaternion.Euler(0, cam_rotation.y - 90, 0);
        //transform.SetPositionAndRotation(p, q);
    }

    void followObjectIK(BulletRobot.IKTarget target)
    {
        if (pybullet != IntPtr.Zero)
        {
            var cmd = NativeMethods.b3CalculateInverseKinematicsCommandInit(pybullet, b3RobotId);
            var targetPosRos = target.targetTF.position.Unity2Ros();
            var targetOrnRos = target.targetTF.rotation;
            //targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
            //targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
            targetOrnRos = targetOrnRos.Unity2Ros();

            double[] targetPos = { targetPosRos.x, targetPosRos.y, targetPosRos.z };
            double[] targetOrn = { targetOrnRos.x, targetOrnRos.y, targetOrnRos.z, targetOrnRos.w };

            NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd, target.endeffectorLinkId, ref targetPos[0], ref targetOrn[0]);
            //NativeMethods.b3CalculateInverseKinematicsAddTargetPurePosition(cmd, target.endeffectorLinkId, ref targetPos[0]);

            var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

            int dofCount = 0;
            double[] jointTargets = new double[freeJoints.Count];
            int bodyId = -1;
            var status = NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);
            status = NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

            cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
            for (int i = 0; i < jointTargets.Length; i++)
            {
                if (!target.kinematicChain.Contains(freeJoints[i])) continue;
                bb.SetJointPosition(ref cmd, b3RobotId, freeJoints[i], jointTargets[i]);
            }
            var st = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
    }

    //void trackHead()
    //{

    //    var eulerAngles = cameraTF.rotation.eulerAngles;
    //    double pitch, roll, yaw;
    //    roll = eulerAngles.x;
    //    pitch = eulerAngles.y - camYOffset;
    //    yaw = eulerAngles.z;

    //    //Debug.Log(BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
    //    IntPtr cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2);
    //    bb.SetJointPosition(ref cmd, b3RobotId, headJointIds[0], BulletBridge.ClipAngle(roll) * Mathf.Deg2Rad);
    //    bb.SetJointPosition(ref cmd, b3RobotId, headJointIds[1], BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
    //    bb.SetJointPosition(ref cmd, b3RobotId, headJointIds[2], BulletBridge.ClipAngle(pitch) * Mathf.Deg2Rad);

    //    var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

    //    lastUpdate = Time.time * 1000;

    //}

    void syncRobotJointStates(ref UrdfRobot robot)
    {

        IntPtr cmd_handle = NativeMethods.b3RequestActualStateCommandInit(pybullet, b3RobotId);
        IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd_handle);

        EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status_handle);

        if (status_type == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
        {
            for (int i = 0; i < robot.GetComponentsInChildren<UrdfJoint>().Length; i++)
            {
                var unityJoint = robot.GetComponentsInChildren<UrdfJoint>()[i];
                if (!jointNames.Contains(unityJoint.JointName)) continue;
                b3JointSensorState state = new b3JointSensorState();

                NativeMethods.b3GetJointState(pybullet, status_handle, b3JointIds[i], ref state);
                var diff = (float)state.m_jointPosition - unityJoint.GetPosition();

                if (unityJoint.JointName.Contains("gripper") || unityJoint.JointName.Contains("gripper_r_joint"))
                {
                    Debug.Log("omitting " + unityJoint.name);
                    continue;

                    //var candidates = from gi in glovesInfo
                    //                 where (gi.isRight && unityJoint.JointName.Contains("rh_")) || (unityJoint.JointName.Contains("lh_") && !gi.isRight)
                    //                 select gi;
                    //var gl = candidates.FirstOrDefault();
                    //if (gl.jointTFs == null) continue;

                    //// var gl = (unityJoint.JointName.Contains("rh_")) ? glovesInfo.Find(x => x.isRight) : glovesInfo.Find(x => !x.isRight);
                    //for (int j = 0; j < gl.thumbPrevSetpoints.Count; j++)
                    //{

                    //    if (b3JointIds[i] == gl.handJointsIds[0][j])
                    //    {
                    //        diff = (float)(state.m_jointPosition - gl.thumbPrevSetpoints[j]);
                    //        gl.thumbPrevSetpoints[j] = state.m_jointPosition;
                    //        //Quaternion rot = Quaternion.AngleAxis(-diff * Mathf.Rad2Deg, new Vector3(0, 0, 1));// unityJoint.UnityJoint.axis);
                    //        //unityJoint.transform.rotation = unityJoint.transform.rotation * rot;
                    //        //unityJoint.GetComponentInChildren<Rigidbody>().transform.rotation = unityJoint.transform.rotation * rot;
                    //        unityJoint.UpdateJointState(diff);
                    //        break;
                    //    }

                    //}
                    //continue;
                }
                Debug.Log(unityJoint.name);
                unityJoint.UpdateJointState(diff);
            }
        }
        else
        {
            Debug.LogWarning("Cannot get robot state");
        }
    }

    void setupRobotJoints()
    {
        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, b3RobotId);
        b3JointNames = new List<string>();
        jointNames = new List<string>();
        var urdfJoints = urdfRobot.GetComponentsInChildren<UrdfJoint>();
        b3JointIds = new List<int>();
        freeJoints = new List<int>();
        excludeFromIkJoints = new List<int>();

        var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
        for (int i = 0; i < b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(pybullet, b3RobotId, i, ref jointInfo);
            b3JointNames.Add(jointInfo.m_jointName);
            if (jointInfo.m_jointType == 0)//JointType.eRevoluteType)
            {
                freeJoints.Add(i);
            }
            if (jointInfo.m_jointName.Contains("gripper_l_finger") || jointInfo.m_jointName.Contains("gripper_r_finger") || jointInfo.m_jointName.Contains("head"))
            {
                excludeFromIkJoints.Add(i);
            }
            //if (jointInfo.m_jointName.Contains("lh"))
            //{
            //    setJointPosition(ref cmd, i, 0);
            //}
        }
        NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);



        foreach (var j in urdfJoints)
        {
            if (b3JointNames.Contains(j.JointName))
            {
                b3JointIds.Add(b3JointNames.IndexOf(j.JointName));
                jointNames.Add(j.JointName);
            }
            else
            {
                Debug.LogWarning("pybullet doesnt know about " + j.JointName);
            }

        }
        //headJointIds = new List<int>
        //{
        //    b3JointIds.ElementAt(jointNames.IndexOf("head_axis0")),
        //    b3JointIds.ElementAt(jointNames.IndexOf("head_axis2")),
        //    b3JointIds.ElementAt(jointNames.IndexOf("head_axis1"))
        //};
        //wristJoints = new List<int>
        //{
        //    b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis0")),
        //    b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis1")),
        //    b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis2"))
        //};
    }
}

using System.Collections;
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
using UnityEngine.Assertions.Must;
using System.Diagnostics.Eventing.Reader;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class BulletBridge : MonoBehaviour
{
    
    private IntPtr pybullet;
    public bool isInitialized;
    private Dictionary<GameObject, int> b3IdMap;
    private float lastUpdate;
    //private List<int> headJointIds;
    //private List<int> freeJoints;
    ////private List<List<int>> handJoints;
    //private List<int> wristJoints;
    //private List<int> excludeFromIkJoints;
    //private int b3RobotId;
    ////private List<double> thumbPrevSetpoints;
    //private double camYOffset;
    //private Vector3 initRobotPosition;
    //private List<GloveInfo> glovesInfo;
    ////private SenseGlove_VirtualHand virtualHand;
    ////private List<List<Transform>> virtualHandJointsTF;
    private bool reset;
    
    struct IKTarget
    {
        public IKTarget(Transform tf, int id, List<int> kc)
        {
            targetTF = tf;
            endeffectorLinkId = id;
            kinematicChain = kc;
        }
        public Transform targetTF;
        public int endeffectorLinkId;
        public List<int> kinematicChain;
    }

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
        
        pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY2);
        
        if (NativeMethods.b3CanSubmitCommand(pybullet) != 1)
        {
            Debug.LogError("Cannot submit commands to pybullet server. Quitting...");
            Application.Quit();
        }
        
        resetSimulation();

        IntPtr cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        //makeKinematic(upperBody.GetComponentsInChildren<Rigidbody>());

        //UrdfRobot = GetComponent<UrdfRobot>();

        //// load URDF models

        ////var cubePath = "C:\\Users\\roboy\\Documents\\code\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\sphere2red.urdf";
        ////var cubeId = loadURDF(cubePath, cube.transform.position, cube.transform.rotation);

        //var tablePath = Application.dataPath + "\\Urdf\\Table\\CheckersTable.urdf";
        //var tableId = loadURDF(tablePath, table.transform.position, table.transform.rotation, 1);     

        //var robotPath = Application.dataPath + "\\Urdf\\roboy3_models\\upper_body\\bullet.urdf";
        //b3RobotId =  loadURDF(robotPath, upperBody.transform.position, upperBody.transform.rotation, 1);

        b3IdMap = new Dictionary<GameObject, int>();
        //{
        //    { table, tableId },
        //    { upperBody, b3RobotId }
        //    //{ cube, cubeId}
        //};

        //var tokenPath = Application.dataPath + "\\Urdf\\token\\black_token.urdf";

        //foreach (var token in tokens)
        //{
        //    makeKinematic(token.GetComponentsInChildren<Rigidbody>());
        //    var tokenId = loadURDF(tokenPath, token.transform.position, token.transform.rotation);
        //    b3IdMap.Add(token, tokenId);
        //}

        //setupRobotJoints();

        //// IK setup
        //IKTargets = new List<IKTarget>()
        //{
            
        //    new IKTarget(rightHandTarget, b3GetLinkId("rh_palm", b3RobotId), getJointKinematicChain("rh_palm", "shoulder_right_link1")),
        //    new IKTarget(leftHandTarget, b3GetLinkId("lh_palm", b3RobotId), getJointKinematicChain("lh_palm", "shoulder_left_link1"))


        //};

        //lastUpdate = Time.time * 1000;

        //camYOffset = cam.transform.rotation.eulerAngles.y;

        //cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        //status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        
        ////Valve.VR.OpenVR.System.ResetSeatedZeroPose();
        //// connect trigger for reset
        //SteamVR_Actions.default_GrabPinch.AddOnStateDownListener(TriggerPressed, SteamVR_Input_Sources.Any);

        //glovesInfo = new List<GloveInfo>();
        //foreach (var glove in senseGloves)
        //{
        //    glovesInfo.Add(new GloveInfo(glove, b3JointIds, jointNames));
        //}

        reset = false;
        //syncPoseUnity2Bullet(upperBody);
        setGravity();
        setRealTimeSimualtion(1);
        isInitialized = true;
        Debug.Log("pybullet connected");
    }

    public void AddGameObject(GameObject go, int bulletId)
    {
        b3IdMap.Add(go, bulletId);
    }

    public int GetIdByObject(GameObject gameObject)
    {
        int id = -1;
        b3IdMap.TryGetValue(gameObject, out id);
        return id;
    }

    public IntPtr GetPhysicsServerPtr()
    {
        if (isInitialized)
        {
            return pybullet;
        }
        else
        {
            Debug.LogError("Trying to access unintialized pybullet server");
            return IntPtr.Zero;
        }
    }

    public IEnumerator WaitForConnection()
    {
        //Print the time of when the function is first called.
        Debug.Log("Waiting for bullet bridge..");

        //yield on a new YieldInstruction that waits for 5 seconds.
        yield return new WaitForSeconds(1);
    }

    // Update is called once per frame
    void Update()
    {
        
        //if (Input.GetKeyDown("space"))
        //{

        //    Debug.LogWarning("reset");
        //    initRobotPosition = upperBody.transform.position;
        //    resetRobotPose();
        //    camYOffset = cam.transform.rotation.eulerAngles.y;
        //    //resetCheckersPose();
        //    syncPoseUnity2Bullet(upperBody);
        //    syncPoseUnity2Bullet(table);

        //    foreach (var token in tokens)
        //    {
        //        syncPoseUnity2Bullet(token);
        //    }

        //    var cubePath = Application.dataPath + "\\Urdf\\sphere2red.urdf";
        //    var cubeId = loadURDF(cubePath, cube.transform.position, cube.transform.rotation);
        //    b3IdMap.Add(cube, cubeId);
        //    reset = true;
        //}

        //if (pybullet != IntPtr.Zero)
        //{
        //    syncRobotJointStates(ref UrdfRobot);
        //    foreach (var token in tokens)
        //        syncPoseBullet2Unity(token);
        //    if (reset) syncPoseBullet2Unity(cube);
        //    //syncBodyState(upperBody);
        //    if (Time.time * 1000 - lastUpdate > 20)
        //    {
        //        trackHead();
        //        if (reset)
        //        {
        //            //leftHandTarget.position = new Vector3(-0.2f * Mathf.Cos(Time.time) +2.0f, -3.0f, 0f + 0.2f * Mathf.Sin(Time.time) + 0.2f).Ros2Unity();
        //            //rightHandTarget.position = new Vector3(-0.2f * Mathf.Cos(Time.time) + 2.0f, -3.0f, 0f + 0.2f * Mathf.Sin(Time.time) + 0.2f).Ros2Unity();
        //            foreach (var target in IKTargets)
        //                followObjectIK(target);
                    
        //        }
        //        foreach (var glove in glovesInfo) trackFingerJoints(glove);
        //    }
        //}

    }

    //void trackFingerJoints(GloveInfo glove)
    //{

    //    var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD); 

    //    for (int i=0; i<glove.handJointsIds.Count; i++)
    //    {
    //        List<Transform> joints = glove.jointTFs[i];// new List<Transform>();
    //        for (int j=0; j< glove.handJointsIds[i].Count; j++)
    //        {   
    //            Quaternion q;
    //            double targetJointPos;
    //            if (i == 0) // thumb is really special
    //            {
    //                switch(j)
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
    //            else {
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

    //void followObjectIK(IKTarget target)
    //{
    //    var cmd = NativeMethods.b3CalculateInverseKinematicsCommandInit(pybullet, b3RobotId);
    //    var targetPosRos = target.targetTF.position.Unity2Ros();
    //    var targetOrnRos = target.targetTF.rotation;
    //    targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
    //    targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
    //    targetOrnRos = targetOrnRos.Unity2Ros();

    //    double[] targetPos = { targetPosRos.x, targetPosRos.y, targetPosRos.z };
    //    double[] targetOrn = { targetOrnRos.x, targetOrnRos.y, targetOrnRos.z, targetOrnRos.w };

    //    NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd, target.endeffectorLinkId, ref targetPos[0], ref targetOrn[0]); 
    //    //NativeMethods.b3CalculateInverseKinematicsAddTargetPurePosition(cmd, target.endeffectorLinkId, ref targetPos[0]);

    //    var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

    //    int dofCount = 0;
    //    double[] jointTargets = new double[freeJoints.Count];
    //    int bodyId = -1;
    //    var status = NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);
    //    status = NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

    //    cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
    //    for (int i=0; i<jointTargets.Length; i++)
    //    {
    //        if (!target.kinematicChain.Contains(freeJoints[i])) continue;
    //        setJointPosition(ref cmd, freeJoints[i], jointTargets[i]);
    //    }
    //    var st = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    //}

    //private void TriggerPressed(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    //{
    //    Debug.LogWarning("reset");
    //    initRobotPosition = upperBody.transform.position;
    //    resetRobotPose();
    //    camYOffset = cam.transform.rotation.eulerAngles.y;
    //    //resetCheckersPose();
    //    syncPoseUnity2Bullet(upperBody);
    //    syncPoseUnity2Bullet(table);

    //    foreach (var token in tokens)
    //    {
    //        syncPoseUnity2Bullet(token);
    //    }
    //}

    //void resetRobotPose()
    //{
    //    var p = - cam.transform.up + cam.transform.forward * -0.1f + cam.transform.position;
    //    var cam_rotation = cam.transform.rotation;
    //    Quaternion q = Quaternion.Euler(0, cam_rotation.y - 90, 0);
    //    upperBody.transform.SetPositionAndRotation(p, q);
    //}

    //void resetCheckersPose()
    //{
    //    var robot_pos = upperBody.transform.position;
    //    var diff = robot_pos - initRobotPosition;
    //    //diff.y -= 0.25f;
    //    //diff.z += 2.0f;
    //    var rotation  = new Quaternion(0, 0, 0, 1f);
    //    table.transform.SetPositionAndRotation(diff+table.transform.position, table.transform.rotation);
    //    foreach (var token in tokens)
    //    {
    //        token.transform.SetPositionAndRotation(diff+token.transform.position, token.transform.rotation);
    //    }

    //}

    //void syncPoseUnity2Bullet(GameObject body)
    //{
    //    int id;
    //    b3IdMap.TryGetValue(body, out id);
    //    if (id>=0)
    //    {
    //        var cmd = NativeMethods.b3CreatePoseCommandInit(pybullet, id);
    //        var pos = RosSharp.TransformExtensions.Unity2Ros(body.transform.position);
    //        var orn = RosSharp.TransformExtensions.Unity2Ros(body.transform.rotation);
    //        if (body == upperBody)
    //        {
    //            NativeMethods.b3CreatePoseCommandSetBasePosition(cmd, pos.x, pos.y, pos.z+0.62);
    //        }  else
    //        {
    //            NativeMethods.b3CreatePoseCommandSetBasePosition(cmd, pos.x, pos.y, pos.z);
    //        }

    //        NativeMethods.b3CreatePoseCommandSetBaseOrientation(cmd, orn.x, orn.y, orn.z, orn.w);
    //        NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    //    }

    //}

    public static void MakeKinematic(Rigidbody[] rbs)
    {
        foreach (var rb in rbs)
        {
            rb.detectCollisions = false;
            rb.isKinematic = true;
        }
    }

    public int LoadURDF(string file, Vector3 p, Quaternion q, int useFixedBase = 0)
    {
        p = RosSharp.TransformExtensions.Unity2Ros(p);
        q = RosSharp.TransformExtensions.Unity2Ros(q);
        var cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, file);
        NativeMethods.b3LoadUrdfCommandSetStartPosition(cmd, p.x, p.y, p.z);
        NativeMethods.b3LoadUrdfCommandSetStartOrientation(cmd, q.x, q.y, q.z, q.w);
        NativeMethods.b3LoadUrdfCommandSetUseFixedBase(cmd, useFixedBase);
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        var bodyId = NativeMethods.b3GetStatusBodyIndex(status);
        return bodyId;
    }

    public struct MyPos
    {
        public double x, y, z;
        public double qx, qy, qz, qw;
    }

    public void SyncPoseBullet2Unity(GameObject body)
    {
        int bodyId = -1;
        b3IdMap.TryGetValue(body, out bodyId);
        if (bodyId >= 0)
        {
            IntPtr cmd_handle =
                NativeMethods.b3RequestActualStateCommandInit(pybullet, bodyId);
            IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd_handle);

            EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status_handle);

            if (status_type == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
            {
                IntPtr p = IntPtr.Zero;
                int numDofQ = 0;
                int numDofU = 0;
                IntPtr inertialFrame = IntPtr.Zero;
                IntPtr actualStateQ = IntPtr.Zero;
                IntPtr actualStateQdot = IntPtr.Zero;
                IntPtr joint_reaction_forces = IntPtr.Zero;

                NativeMethods.b3GetStatusActualState(
                status_handle, ref bodyId, ref numDofQ, ref numDofU,
                ref inertialFrame, ref actualStateQ,
                ref actualStateQdot, ref joint_reaction_forces);

                MyPos mpos = (MyPos)Marshal.PtrToStructure(actualStateQ, typeof(MyPos));
                Vector3 pos = new Vector3((float)mpos.x, (float)mpos.y, (float)mpos.z);
                Quaternion orn = new Quaternion((float)mpos.qx, (float)mpos.qy, (float)mpos.qz, (float)mpos.qw);

                pos = RosSharp.TransformExtensions.Ros2Unity(pos);
                orn = RosSharp.TransformExtensions.Ros2Unity(orn);
                var tf = body.transform.position;
                body.transform.SetPositionAndRotation(pos, orn);
            }
        }
    }

    //void syncRobotJointStates(ref UrdfRobot robot)
    //{

    //    IntPtr cmd_handle = NativeMethods.b3RequestActualStateCommandInit(pybullet, b3RobotId);
    //    IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd_handle);

    //    EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status_handle);

    //    if (status_type == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
    //    {
    //        for (int i = 0; i < robot.GetComponentsInChildren<UrdfJoint>().Length; i++)
    //        {
    //            var unityJoint = robot.GetComponentsInChildren<UrdfJoint>()[i];
    //            if (!jointNames.Contains(unityJoint.JointName)) continue;
    //            b3JointSensorState state = new b3JointSensorState();

    //            NativeMethods.b3GetJointState(pybullet, status_handle, b3JointIds[i], ref state);
    //            var diff = (float)state.m_jointPosition - unityJoint.GetPosition();

    //            if (unityJoint.JointName.Contains("THJ"))
    //            {

    //                var candidates = from gi in glovesInfo
    //                            where (gi.isRight && unityJoint.JointName.Contains("rh_")) || (unityJoint.JointName.Contains("lh_") && !gi.isRight)
    //                            select gi;
    //                var gl = candidates.FirstOrDefault();
    //                if (gl.jointTFs == null) continue;

    //               // var gl = (unityJoint.JointName.Contains("rh_")) ? glovesInfo.Find(x => x.isRight) : glovesInfo.Find(x => !x.isRight);
    //                for (int j = 0; j < gl.thumbPrevSetpoints.Count; j++)
    //                {

    //                    if (b3JointIds[i] == gl.handJointsIds[0][j])
    //                    {
    //                        diff = (float)(state.m_jointPosition - gl.thumbPrevSetpoints[j]);
    //                        gl.thumbPrevSetpoints[j] = state.m_jointPosition;
    //                        //Quaternion rot = Quaternion.AngleAxis(-diff * Mathf.Rad2Deg, new Vector3(0, 0, 1));// unityJoint.UnityJoint.axis);
    //                        //unityJoint.transform.rotation = unityJoint.transform.rotation * rot;
    //                        //unityJoint.GetComponentInChildren<Rigidbody>().transform.rotation = unityJoint.transform.rotation * rot;
    //                        unityJoint.UpdateJointState(diff);
    //                        break;
    //                    }

    //                }
    //                continue;
    //            }
    //            unityJoint.UpdateJointState(diff);
    //        }
    //    }
    //    else
    //    {
    //        Debug.LogWarning("Cannot get robot state");
    //    }
    //}

    //void trackHead()
    //{

    //    var eulerAngles = cam.transform.rotation.eulerAngles;
    //    double pitch, roll, yaw; 
    //    roll = eulerAngles.x;
    //    pitch = eulerAngles.y - camYOffset;
    //    yaw = eulerAngles.z;

    //    Debug.Log(clipAngle(yaw) * Mathf.Deg2Rad);
    //    IntPtr cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2);
    //    setJointPosition(ref cmd, headJointIds[0], clipAngle(roll) * Mathf.Deg2Rad);
    //    setJointPosition(ref cmd, headJointIds[1], clipAngle(yaw) * Mathf.Deg2Rad);
    //    setJointPosition(ref cmd, headJointIds[2], clipAngle(pitch) * Mathf.Deg2Rad);

    //    var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

    //    lastUpdate = Time.time * 1000;

    //}

    public void SetJointPosition(ref IntPtr cmd, int bodyId, int jointIndex, double targetPositionRad)
    {
        b3JointInfo ji = new b3JointInfo();
        NativeMethods.b3GetJointInfo(pybullet, bodyId, jointIndex, ref ji);
        NativeMethods.b3JointControlSetDesiredPosition(cmd, ji.m_qIndex, targetPositionRad);
        NativeMethods.b3JointControlSetKp(cmd, ji.m_uIndex, 0.1);
        NativeMethods.b3JointControlSetDesiredVelocity(cmd, ji.m_uIndex, 0);
        NativeMethods.b3JointControlSetKd(cmd, ji.m_uIndex, 1.0);
        NativeMethods.b3JointControlSetMaximumForce(cmd, ji.m_uIndex, 100000.0);

    }

    public List<int> GetJointKinematicChain(int bodyId, string ef, string root)
    {
        
        var start = b3GetLinkId(root, bodyId);
        var efLinkId = b3GetLinkId(ef, bodyId);
        var chain = new List<int>();
        for (int i=start; i<=efLinkId; i++)
        {
            chain.Add(i);
            //TODO check if joint is movable
        }
        return chain;
    }

    //void setupRobotJoints()
    //{
    //    b3JointInfo jointInfo = new b3JointInfo();
    //    var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, b3RobotId);
    //    b3JointNames = new List<string>();
    //    jointNames = new List<string>();
    //    var urdfJoints = UrdfRobot.GetComponentsInChildren<UrdfJoint>();
    //    b3JointIds = new List<int>();
    //    freeJoints = new List<int>();
    //    excludeFromIkJoints = new List<int>();

    //    var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, (int)EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
    //    for (int i = 0; i < b3JointsNum; i++)
    //    {
    //        NativeMethods.b3GetJointInfo(pybullet, b3RobotId, i, ref jointInfo);
    //        b3JointNames.Add(jointInfo.m_jointName);
    //        if (jointInfo.m_jointType == 0)//JointType.eRevoluteType)
    //        {
    //            freeJoints.Add(i);
    //        }
    //        if (jointInfo.m_jointName.Contains("rh_") || jointInfo.m_jointName.Contains("lh_") || jointInfo.m_jointName.Contains("head"))
    //        {
    //            excludeFromIkJoints.Add(i);
    //        }
    //        //if (jointInfo.m_jointName.Contains("lh"))
    //        //{
    //        //    setJointPosition(ref cmd, i, 0);
    //        //}
    //    }
    //    NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);



    //    foreach (var j in urdfJoints)
    //    {
    //        if (b3JointNames.Contains(j.JointName))
    //        {
    //            b3JointIds.Add(b3JointNames.IndexOf(j.JointName));
    //            jointNames.Add(j.JointName);
    //        } else
    //        {
    //            Debug.LogWarning("pybullet doesnt know about " + j.JointName);
    //        }

    //    }
    //    headJointIds = new List<int>
    //    {
    //        b3JointIds.ElementAt(jointNames.IndexOf("head_axis0")),
    //        b3JointIds.ElementAt(jointNames.IndexOf("head_axis2")),
    //        b3JointIds.ElementAt(jointNames.IndexOf("head_axis1"))
    //    };
    //    wristJoints = new List<int>
    //    {
    //        b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis0")),
    //        b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis1")),
    //        b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis2"))
    //    };


    //    //handJoints = new List<List<int>>(){
    //    //    new List<int> {
    //    //        b3JointIds.ElementAt(b3JointNames.IndexOf("rh_THJ5")),
    //    //        b3JointIds.ElementAt(b3JointNames.IndexOf("rh_THJ4")),
    //    //        b3JointIds.ElementAt(b3JointNames.IndexOf("rh_THJ3")),
    //    //        b3JointIds.ElementAt(b3JointNames.IndexOf("rh_THJ2"))
    //    //        //b3JointIds.ElementAt(jointNames.IndexOf("rh_THJ1"))

    //    //    },
    //    //    new List<int> {
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_FFJ3")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_FFJ2")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_FFJ1"))
    //    //    },
    //    //    new List<int> {
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_MFJ3")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_MFJ2")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_MFJ1"))
    //    //    },
    //    //    new List<int> {
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_RFJ3")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_RFJ2")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_RFJ1"))
    //    //    },
    //    //    new List<int> {
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_LFJ3")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_LFJ2")),
    //    //        b3JointIds.ElementAt(jointNames.IndexOf("rh_LFJ1"))
    //    //    }
    //    //};
    //    //thumbPrevSetpoints = new List<double>(new double[handJoints[0].Count]);


    //}

    public int b3GetLinkId(string name, int bodyId)
    {
        b3JointInfo ji = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, bodyId);
        for (int i=0; i<b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(pybullet, bodyId, i, ref ji);
            if (ji.m_linkName == name)
            {
                return i;
            }
        }
        Debug.LogError("Could not find link id with name " + name);
        return -1;
        
    }

    public string b3GetJointName(int jointId, int bodyId)
    {
        b3JointInfo ji = new b3JointInfo();
        NativeMethods.b3GetJointInfo(pybullet, bodyId, jointId, ref ji);
        return ji.m_jointName;
    }

    public void resetSimulation()
    {
        IntPtr cmd = NativeMethods.b3InitResetSimulationCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    }
    public void stepSimulation()
    {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitStepSimulationCommand(pybullet);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
    }

    public void setRealTimeSimualtion(int flag)
    {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitPhysicsParamCommand(pybullet);
            NativeMethods.b3PhysicsParamSetRealTimeSimulation(cmd, flag);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
    }

    public void setGravity()
    {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr command = NativeMethods.b3InitPhysicsParamCommand(pybullet);
            NativeMethods.b3PhysicsParamSetGravity(command, 0, 0, -9.8);
            IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, command);
        }
       
    }

    public static double ClipAngle(double angle)
    {
        if (angle > 180)
        {
            angle -= 360;
        }
        else if (angle < -180)
        {
            angle += 360;
        }
        return angle;
    }

    private void OnDestroy()
    {
        if (pybullet != IntPtr.Zero)
        {
            NativeMethods.b3DisconnectSharedMemory(pybullet);
        }
    }
}

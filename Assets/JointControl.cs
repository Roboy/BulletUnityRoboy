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

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class JointControl : MonoBehaviour
{
    public GameObject upperBody;
    public GameObject table;
    public GameObject head;
    public GameObject glove;
    public GameObject cube;
    public List<GameObject> tokens;
    public Camera cam;
    public Transform wristTransform;
    public GameObject IKTarget;
    private UrdfRobot UrdfRobot;
    private UrdfJoint joint;
    private IntPtr pybullet;
    private Dictionary<GameObject, int> b3IdMap;
    private List<int> b3JointIds;
    private List<string> jointNames; // synced with b3JointIds
    private float lastHeadUpdate;
    private List<int> headJointIds;
    private List<int> freeJoints;
    private List<List<int>> handJoints;
    private List<int> wristJoints;
    private int b3RobotId;
    private double setpoint;
    private double camYOffset;
    private List<double> wristOffset;
    private Vector3 initRobotPosition;
    private SenseGlove_VirtualHand virtualHand;
    private bool reset;
    private List<int> excludeFromIkJoints;


    // Start is called before the first frame update
    void Start()
    {
        
        pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY2);
        Debug.Log(NativeMethods.b3CanSubmitCommand(pybullet));


        resetSimulation();

        IntPtr cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        upperBody = GameObject.Find("upper_body");
        makeKinematic(upperBody.GetComponentsInChildren<Rigidbody>());
        //var cam_pos = cam.transform.position;
        //cam_pos.y -= 0.9f;
        //var cam_rotation = cam.transform.rotation;
        ////cam_rotation.eulerAngles.Set(0, cam_rotation.eulerAngles.y-90, 0);
        //Quaternion q = new Quaternion(0, -0.707f, 0, 0.707f);
        //cam_rotation = q * cam_rotation;
        ////q = new Quaternion(0, 0, 1.0f, 1.0f);
        ////cam_rotation = q * cam_rotation;
        //upperBody.transform.SetPositionAndRotation(cam_pos, cam_rotation);



        UrdfRobot = GetComponent<UrdfRobot>();

        // load URDF models

        //var cubePath = "C:\\Users\\roboy\\Documents\\code\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\sphere2red.urdf";
        //var cubeId = loadURDF(cubePath, cube.transform.position, cube.transform.rotation);

        var tablePath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\Table\\CheckersTable.urdf";
        var tableId = loadURDF(tablePath, table.transform.position, table.transform.rotation, 1);     

        var robotPath = "C:\\Users\\roboy\\Documents\\code\\roboy3_models\\upper_body\\bullet.urdf";
        b3RobotId =  loadURDF(robotPath, upperBody.transform.position, upperBody.transform.rotation, 1);
        
        


        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, b3RobotId);
        List<string> b3JointNames = new List<string>();
        jointNames = new List<string>();
        var urdfJoints = UrdfRobot.GetComponentsInChildren<UrdfJoint>();
        b3JointIds = new List<int> ();
        freeJoints = new List<int>();
        excludeFromIkJoints = new List<int>();

        cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2); 
        for (int i=0; i<b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(pybullet, b3RobotId, i, ref jointInfo);
            b3JointNames.Add(jointInfo.m_jointName);
            if ((JointType) jointInfo.m_jointType != JointType.eFixedType)
            {
                freeJoints.Add(i);
            }
            if (jointInfo.m_jointName.Contains("rh") || jointInfo.m_jointName.Contains("lh") || jointInfo.m_jointName.Contains("head"))
            {
                excludeFromIkJoints.Add(i);
            }
            if (jointInfo.m_jointName.Contains("lh"))
            {
                setJointPosition(ref cmd, i, 0);
            }
        }
        NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
         
        
        foreach(var j in urdfJoints)
        {
            if (b3JointNames.Contains(j.JointName))
            {
                b3JointIds.Add(b3JointNames.IndexOf(j.JointName));
                jointNames.Add(j.JointName);
            }

        }
        headJointIds = new List<int>
        {
            b3JointIds.ElementAt(jointNames.IndexOf("head_axis0")),
            b3JointIds.ElementAt(jointNames.IndexOf("head_axis2")),
            b3JointIds.ElementAt(jointNames.IndexOf("head_axis1"))
        };
        wristJoints = new List<int>
        {
            b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis0")),
            b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis1")),
            b3JointIds.ElementAt(jointNames.IndexOf("wrist_right_axis2"))
        };
        wristOffset = new List<double>
        {
            0,0,0
        };


        handJoints = new List<List<int>>(){
            new List<int> {
                b3JointIds.ElementAt(jointNames.IndexOf("rh_THJ4")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_THJ3")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_THJ2")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_THJ1"))

            },
        new List<int> {
                b3JointIds.ElementAt(jointNames.IndexOf("rh_FFJ3")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_FFJ2")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_FFJ1")) 
            },
            new List<int> {
                b3JointIds.ElementAt(jointNames.IndexOf("rh_MFJ3")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_MFJ2")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_MFJ1")) 
            },
            new List<int> {
                b3JointIds.ElementAt(jointNames.IndexOf("rh_RFJ3")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_RFJ2")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_RFJ1"))
            },
            new List<int> {
                b3JointIds.ElementAt(jointNames.IndexOf("rh_LFJ3")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_LFJ2")),
                b3JointIds.ElementAt(jointNames.IndexOf("rh_LFJ1"))
            }
        };

        b3IdMap = new Dictionary<GameObject, int>
        {
            { table, tableId },
            { upperBody, b3RobotId }
            //{ cube, cubeId}
        };

        var tokenPath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\tokenbk\\token.urdf";

        foreach (var token in tokens)
        {
            if (token.name.Contains("black"))
            {
                tokenPath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\token\\black_token.urdf";
            }
            makeKinematic(token.GetComponentsInChildren<Rigidbody>());
            var tokenId = loadURDF(tokenPath, token.transform.position, token.transform.rotation);
            b3IdMap.Add(token, tokenId);
        }

        
        lastHeadUpdate = Time.time * 1000;
        setpoint = 0;

        camYOffset = cam.transform.rotation.eulerAngles.y;

        cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        Valve.VR.OpenVR.System.ResetSeatedZeroPose();
        SteamVR_Actions.default_GrabPinch.AddOnStateDownListener(TriggerPressed, SteamVR_Input_Sources.Any);

        virtualHand = glove.GetComponent<SenseGlove_VirtualHand>();
        //virtualHand.CollectFingerJoints();
        //virtualHand.CollectCorrections();

        reset = false;
        syncPoseUnity2Bullet(upperBody);
        setGravity();
        setRealTimeSimualtion(1);
        UrdfRobot = GetComponent<UrdfRobot>();

    }

    //private void FixedUpdate()
    //{
        
    //    if (pybullet != IntPtr.Zero)
    //    {
    //        syncRobotJointStates(ref UrdfRobot);
    //        foreach (var token in tokens)
    //            syncBodyState(token);
    //        //syncBodyState(table);
    //        //syncBodyState(upperBody);
    //        if (Time.time * 1000 - lastHeadUpdate > 20)
    //        {
    //            trackHead();
    //            if (reset) followObjectIK(IKTarget);
    //            trackFingerJoints();
    //        }
    //    }
    //}

    // Update is called once per frame
    void Update()
    {
        
        //Debug.Log(hand.indexFingerJoints[2].rotation.eulerAngles);
        //var q = new Quaternion(0, 1.57f, 0, 1.57f);
        //cam.transform.SetPositionAndRotation(camHolder.transform.position, q*cam.transform.rotation);
        if (Input.GetKeyDown("space"))
        {
            
            Debug.LogWarning("reset");
            initRobotPosition = upperBody.transform.position;
            resetRobotPose();
            camYOffset = cam.transform.rotation.eulerAngles.y;
            //resetCheckersPose();
            syncPoseUnity2Bullet(upperBody);
            syncPoseUnity2Bullet(table);
           
           
            foreach(var token in tokens) {
                syncPoseUnity2Bullet(token);
            }
            wristOffset[0] = IKTarget.transform.rotation.eulerAngles.x;
            wristOffset[1] = IKTarget.transform.rotation.eulerAngles.y;
            wristOffset[2] = IKTarget.transform.rotation.eulerAngles.z;
            var cubePath = "C:\\Users\\roboy\\Documents\\code\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\sphere2red.urdf";
            var cubeId = loadURDF(cubePath, cube.transform.position, cube.transform.rotation);
            b3IdMap.Add(cube, cubeId);
            reset = true;
        }
        UrdfRobot = GetComponent<UrdfRobot>();
        if (pybullet != IntPtr.Zero)
        {
            syncRobotJointStates(ref UrdfRobot);
            foreach (var token in tokens)
                syncBodyState(token);
            if (reset) syncBodyState(cube);
            //syncBodyState(upperBody);
            if (Time.time * 1000 - lastHeadUpdate > 10)
            {
                trackHead();
                if (reset) followObjectIK(IKTarget);
                trackFingerJoints();
            }
        }

    }

    void trackFingerJoints()
    {
        var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2);
        for (int i=0; i<handJoints.Count; i++)
        {
            List<Transform> joints = new List<Transform>();
            for (int j=0; j<handJoints[i].Count; j++)
            {
                switch (i)
                {
                    case 0:
                        joints = virtualHand.thumbJoints;
                        break;
                    case 1:
                        joints = virtualHand.indexFingerJoints;
                        break;
                    case 2:
                        joints = virtualHand.middleFingerJoints;
                        break;
                    case 3:
                        joints = virtualHand.ringFingerJoints;
                        break;
                    case 4:
                        joints = virtualHand.pinkyJoints;
                        break;

                    default:
                        Debug.LogError("unknown finger");
                        break;

                        
                }
                Quaternion q;
                if (i==0 && j>0)
                {
                    q = (joints[j-1].rotation * virtualHand.fingerCorrection[i][j-1]).Unity2Ros();
                } else
                {
                    q = (joints[j].rotation * virtualHand.fingerCorrection[i][j]).Unity2Ros();
                }
                
                //var q = joints[j].rotation.Unity2Ros();
                if (i == 0)
                {
                    
                    if (j==0)
                    {
                        //Debug.LogWarning("x: " + clipAngle(q.eulerAngles.x) + "\t y: " + clipAngle(q.eulerAngles.y) + "\t z: " + clipAngle(q.eulerAngles.z));
                        //Debug.Log(clipAngle(q.eulerAngles.y) * Mathf.Deg2Rad);
                        setJointPosition(ref cmd, handJoints[i][j], clipAngle(q.eulerAngles.y) * Mathf.Deg2Rad+0.6);
                        continue;
                    }
                    if (j==1)
                    {
                        //Debug.Log(clipAngle(q.eulerAngles.x) +"\t "+ clipAngle(q.eulerAngles.y) + "\t" + clipAngle(q.eulerAngles.z));
                        setJointPosition(ref cmd, handJoints[i][j], clipAngle(-q.eulerAngles.z) * Mathf.Deg2Rad);
                        continue;
                        //Debug.LogWarning(clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad);
                    }
                    if (j==3)
                    {
                        //Debug.Log(clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad);
                        //setJointPosition(ref cmd, handJoints[i][j], clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad);
                        continue;
                    }

                    setJointPosition(ref cmd, handJoints[i][j], clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad);
                    continue;
                }
                if (j==0)
                {
                    //if (i==0)
                    //{
                    //    Debug.LogWarning(clipAngle(q.eulerAngles.x) * Mathf.Deg2Rad);
                    //    setJointPosition(ref cmd, handJoints[i][j], clipAngle(-q.eulerAngles.z) * Mathf.Deg2Rad);
                    //}
                    //Debug.LogWarning(clipAngle(-q.eulerAngles.z) * Mathf.Deg2Rad);
                    setJointPosition(ref cmd, handJoints[i][j], clipAngle(-q.eulerAngles.z+100) * Mathf.Deg2Rad);
                    continue;
                }
                setJointPosition(ref cmd, handJoints[i][j], clipAngle(-q.eulerAngles.z)*Mathf.Deg2Rad);
            }
        }
        NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    }

    double clipAngle(double angle)
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

    void followObjectIK(GameObject target)
    {
        var cmd = NativeMethods.b3CalculateInverseKinematicsCommandInit(pybullet, b3RobotId);
        var targetPosRos = target.transform.position.Unity2Ros();
        //target.transform.Rotate(90, 180, 0);

        var targetOrnRos = wristTransform.rotation;//.Unity2Ros(); // target.transform.rotation.Unity2Ros();
        targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
        targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
        //targetOrnRos *= Quaternion.AngleAxis(-180, new Vector3(0, 0, 1));
        targetOrnRos = targetOrnRos.Unity2Ros();
        targetOrnRos = Quaternion.Euler(targetOrnRos.eulerAngles.x, targetOrnRos.eulerAngles.y, targetOrnRos.eulerAngles.z);
        //var newEuler = new Vector3(-targetOrnRos.eulerAngles.x , targetOrnRos.eulerAngles.y, -targetOrnRos.eulerAngles.z);
        //targetOrnRos = Quaternion.Euler(newEuler).Unity2Ros();
        double[] targetPos = { targetPosRos.x, targetPosRos.y, targetPosRos.z };
        double[] targetOrn = { targetOrnRos.x, targetOrnRos.y, targetOrnRos.z, targetOrnRos.w };
        int[] dofCount = new int[2];
        double[] jointTargets = new double[100];
        int bodyId =-1;
        NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd, 8, ref targetPos[0], ref targetOrn[0]);
        //NativeMethods.b3CalculateInverseKinematicsAddTargetPurePosition(cmd, 7, ref targetPos[0]);
        var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount[0], ref jointTargets[0]);
        cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2);
        foreach (var id in freeJoints)
        {
            if (excludeFromIkJoints.Contains(id))//headJointIds.Contains(id) ) 
            {
                continue;
            }
            setJointPosition(ref cmd, id, jointTargets[id]);
        }
        
        
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    }

    private void TriggerPressed(SteamVR_Action_Boolean fromAction, SteamVR_Input_Sources fromSource)
    {
        Debug.LogWarning("reset");
        initRobotPosition = upperBody.transform.position;
        resetRobotPose();
        camYOffset = cam.transform.rotation.eulerAngles.y;
        //resetCheckersPose();
        syncPoseUnity2Bullet(upperBody);
        syncPoseUnity2Bullet(table);

        foreach (var token in tokens)
        {
            syncPoseUnity2Bullet(token);
        }
    }

    void resetRobotPose()
    {
        var p = - cam.transform.up + cam.transform.forward * -0.1f + cam.transform.position;
        var cam_rotation = cam.transform.rotation;
        var q = Quaternion.Euler(0, cam_rotation.y-90, 0);
        upperBody.transform.SetPositionAndRotation(p, q);
    }

    void resetCheckersPose()
    {
        var robot_pos = upperBody.transform.position;
        var diff = robot_pos - initRobotPosition;
        //diff.y -= 0.25f;
        //diff.z += 2.0f;
        var rotation  = new Quaternion(0, 0, 0, 1f);
        table.transform.SetPositionAndRotation(diff+table.transform.position, table.transform.rotation);
        foreach (var token in tokens)
        {
            token.transform.SetPositionAndRotation(diff+token.transform.position, token.transform.rotation);
        }

    }

    void syncPoseUnity2Bullet(GameObject body)
    {
        int id;
        b3IdMap.TryGetValue(body, out id);
        if (id>=0)
        {
            var cmd = NativeMethods.b3CreatePoseCommandInit(pybullet, id);
            var pos = RosSharp.TransformExtensions.Unity2Ros(body.transform.position);
            var orn = RosSharp.TransformExtensions.Unity2Ros(body.transform.rotation);
            if (body == upperBody)
            {
                NativeMethods.b3CreatePoseCommandSetBasePosition(cmd, pos.x, pos.y, pos.z+0.62);
            }  else
            {
                NativeMethods.b3CreatePoseCommandSetBasePosition(cmd, pos.x, pos.y, pos.z);
            }
            
            NativeMethods.b3CreatePoseCommandSetBaseOrientation(cmd, orn.x, orn.y, orn.z, orn.w);
            NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
        
    }

    void makeKinematic(Rigidbody[] rbs)
    {
        foreach (var rb in rbs)
        {
            //if(rb.name.Contains("rh_th")) {
            //    Debug.Log(rb.name);
            //}
            //rb.interpolation = RigidbodyInterpolation.Extrapolate;
            //rb.inertiaTensor = new Vector3(50,50,50);
            //rb.centerOfMass = new Vector3(0, 0, 0);
            rb.detectCollisions = false;
            rb.isKinematic = true;
        }
    }

    int loadURDF(string file, Vector3 p, Quaternion q, int useFixedBase=0) 
    {
        var cmd = NativeMethods.b3CreateCollisionShapeCommandInit(pybullet);
        p = RosSharp.TransformExtensions.Unity2Ros(p);
        q = RosSharp.TransformExtensions.Unity2Ros(q);
        cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, file);
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

    void syncBodyState(GameObject body)
    {
        int bodyId = -1;
        b3IdMap.TryGetValue(body, out bodyId);
        if (bodyId >= 0 )
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

    void syncRobotJointStates(ref UrdfRobot robot)
    {
        
        IntPtr cmd_handle =
                NativeMethods.b3RequestActualStateCommandInit(pybullet, b3RobotId);
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
                if (unityJoint.JointName.Contains("rh_THJ"))
                {
                    unityJoint.UpdateJointState(0);
                    continue;
                }
                unityJoint.UpdateJointState(diff);


               
            }

        }
        else
        {
            Debug.LogWarning("Cannot get robot state");
        }
    }

    void trackHead()
    {
        //if (Time.time * 1000 - lastHeadUpdate > 100)
        //{
            var eulerAngles = cam.transform.rotation.eulerAngles;
            var roll = eulerAngles.x;
            var pitch = eulerAngles.y-camYOffset;
            var yaw = eulerAngles.z;
            if (pitch > 180)
            {
                pitch -= 360;
            } else if(pitch < -180)
            {
                pitch += 360;
            }
            if (roll > 100)
            {
                roll -= 360;
            }
            if (yaw > 200)
            {
                yaw -= 360;
            }

            IntPtr cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2);
            setJointPosition(ref cmd, headJointIds[0], roll * Mathf.Deg2Rad);
            //Debug.LogWarning(roll + "\t" + roll * Mathf.Deg2Rad);
            setJointPosition(ref cmd, headJointIds[1], yaw * Mathf.Deg2Rad);
            setJointPosition(ref cmd, headJointIds[2], pitch * Mathf.Deg2Rad);

            var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
            
            lastHeadUpdate = Time.time * 1000;
           
       // }
    }

    void setJointPosition(ref IntPtr cmd, int jointIndex, double targetPositionRad)
    {
        b3JointInfo ji = new b3JointInfo();
        NativeMethods.b3GetJointInfo(pybullet, b3RobotId, jointIndex, ref ji);
        NativeMethods.b3JointControlSetDesiredPosition(cmd, ji.m_qIndex, targetPositionRad);
        NativeMethods.b3JointControlSetKp(cmd, ji.m_uIndex, 0.1);
        NativeMethods.b3JointControlSetDesiredVelocity(cmd, ji.m_uIndex, 0);
        NativeMethods.b3JointControlSetKd(cmd, ji.m_uIndex, 1.0);
        NativeMethods.b3JointControlSetMaximumForce(cmd, ji.m_uIndex, 100000.0);

    }

    void resetSimulation()
    {
        IntPtr cmd = NativeMethods.b3InitResetSimulationCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    }
    void stepSimulation()
    {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitStepSimulationCommand(pybullet);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
    }

    void setRealTimeSimualtion(int flag)
    {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitPhysicsParamCommand(pybullet);
            NativeMethods.b3PhysicsParamSetRealTimeSimulation(cmd, flag);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        }
    }

    void setGravity()
    {
        if (pybullet != IntPtr.Zero)
        {
            IntPtr command = NativeMethods.b3InitPhysicsParamCommand(pybullet);
            NativeMethods.b3PhysicsParamSetGravity(command, 0, 0, -9.8);
            IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, command);
        }
       
    }

    private void OnDestroy()
    {
        if (pybullet != IntPtr.Zero)
        {
            NativeMethods.b3DisconnectSharedMemory(pybullet);
        }
    }
}

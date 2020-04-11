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

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class JointControl : MonoBehaviour
{
    public GameObject upperBody;
    public GameObject table;
    public GameObject head;
    public List<GameObject> tokens;
    public Camera cam;
    public GameObject camHolder;
    private UrdfRobot UrdfRobot;
    private UrdfJoint joint;
    private IntPtr pybullet;
    private Dictionary<GameObject, int> b3IdMap;
    private List<int> b3JointIds;
    private List<string> jointNames; // synced with b3JointIds
    private float lastHeadUpdate;
    private List<int> headJointIds;
    private int b3RobotId;
    private double setpoint;
    private double camYOffset;
    private Vector3 initRobotPosition;
    

    // Start is called before the first frame update
    void Start()
    {
        
        pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY2);
        Debug.Log(NativeMethods.b3CanSubmitCommand(pybullet));


        //resetSimulation();

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

        var cubePath = "C:\\Users\\roboy\\Documents\\code\\bullet3\\examples\\pybullet\\gym\\pybullet_data\\cube_no_rotation.urdf";
        //var cubeId = loadURDF(cubePath, cube.transform.position, cube.transform.rotation, 1);

        var tablePath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\Table\\CheckersTable.urdf";
        var tableId = loadURDF(tablePath, table.transform.position, table.transform.rotation, 1);     

        var robotPath = "C:\\Users\\roboy\\Documents\\code\\roboy3_models\\upper_body\\bullet.urdf";
        b3RobotId = 0;// loadURDF(robotPath, upperBody.transform.position, upperBody.transform.rotation, 1);
        
        


        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, b3RobotId);
        List<string> b3JointNames = new List<string>();
        jointNames = new List<string>();
        var urdfJoints = UrdfRobot.GetComponentsInChildren<UrdfJoint>();
        b3JointIds = new List<int> ();
        
        for (int i=0; i<b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(pybullet, b3RobotId, i, ref jointInfo);
            b3JointNames.Add(jointInfo.m_jointName);
        }
        
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


        b3IdMap = new Dictionary<GameObject, int>
        {
            { table, tableId },
            { upperBody, b3RobotId }
        };

        var tokenPath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\token\\token.urdf";

        foreach (var token in tokens)
        {
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
        syncPoseUnity2Bullet(upperBody);
        setGravity();
        setRealTimeSimualtion(1);

    }

    // Update is called once per frame
    void Update()
    {
        //var q = new Quaternion(0, 1.57f, 0, 1.57f);
        //cam.transform.SetPositionAndRotation(camHolder.transform.position, q*cam.transform.rotation);
        if (Input.GetKeyDown("space"))
        {
            Debug.LogWarning("reset");
            initRobotPosition = upperBody.transform.position;
            resetRobotPose();
            //resetCheckersPose();
            syncPoseUnity2Bullet(upperBody);
            syncPoseUnity2Bullet(table);
           
            foreach(var token in tokens) {
                syncPoseUnity2Bullet(token);
            }
            
        }
        UrdfRobot = GetComponent<UrdfRobot>();
        if (pybullet != IntPtr.Zero)
        {
            syncRobotJointStates(ref UrdfRobot);
            foreach (var token in tokens)
                syncBodyState(token);
            //syncBodyState(table);
            //syncBodyState(upperBody);
            trackHead();
            
        }

    }

    void resetRobotPose()
    {
        var cam_pos = cam.transform.position;
        cam_pos.y -= 0.9f;
        var cam_rotation = cam.transform.rotation;
        Quaternion q = new Quaternion(0, -0.707f, 0, 0.707f);
        cam_rotation = q * cam_rotation;
        upperBody.transform.SetPositionAndRotation(cam_pos, cam_rotation);
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
                NativeMethods.b3CreatePoseCommandSetBasePosition(cmd, pos.x, pos.y, pos.z+0.6);
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
            //rb.detectCollisions = false;
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
                if (unityJoint.JointName.Contains("TH") || !jointNames.Contains(unityJoint.JointName)) continue;
                
                
                
                b3JointSensorState state = new b3JointSensorState();
  
                NativeMethods.b3GetJointState(pybullet, status_handle, b3JointIds[i], ref state);
                
                unityJoint.UpdateJointState((float)state.m_jointPosition - unityJoint.GetPosition());
            }

            //Debug.Log(state.m_jointPosition);
        }
        else
        {
            Debug.LogWarning("Cannot get robot state");
        }
    }

    void trackHead()
    {
        if (Time.time * 1000 - lastHeadUpdate > 100)
        {
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
            setJointPosition(ref cmd, headJointIds[0], roll);
            setJointPosition(ref cmd, headJointIds[1], yaw);
            setJointPosition(ref cmd, headJointIds[2], pitch);

            var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
            
            lastHeadUpdate = Time.time * 1000;
           
        }
    }

    void setJointPosition(ref IntPtr cmd, int jointIndex, double targetPositionDeg)
    {
        b3JointInfo ji = new b3JointInfo();
        NativeMethods.b3GetJointInfo(pybullet, b3RobotId, jointIndex, ref ji);
        NativeMethods.b3JointControlSetDesiredPosition(cmd, ji.m_qIndex, targetPositionDeg * Mathf.Deg2Rad);
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

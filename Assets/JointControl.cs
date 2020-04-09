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

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class JointControl : MonoBehaviour
{
    public GameObject upperBody;
    public GameObject table;
    public GameObject token;
    private UrdfRobot UrdfRobot;
    private UrdfJoint joint;
    private IntPtr pybullet;
    private Dictionary<GameObject, int> b3IdMap;
    private List<int> b3JointIds;

    // Start is called before the first frame update
    void Start()
    {
        pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY2);
        Debug.Log(NativeMethods.b3CanSubmitCommand(pybullet));
        IntPtr cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);


        upperBody = GameObject.Find("upper_body");
        makeKinematic(upperBody.GetComponentsInChildren<Rigidbody>());
        makeKinematic(token.GetComponentsInChildren<Rigidbody>());

        UrdfRobot = GetComponent<UrdfRobot>();

        // load URDF models
        var tablePath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\Table\\CheckersTable.urdf";
        var tableId = loadURDF(tablePath, table.transform.position, table.transform.rotation, 1);

        var tokenPath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\token\\token.urdf";
        var tokenId = loadURDF(tokenPath, token.transform.position, token.transform.rotation);

        var robotPath = "C:\\Users\\roboy\\Documents\\code\\roboy3_models\\upper_body\\bullet.urdf";
        var robotId = loadURDF(robotPath, upperBody.transform.position, upperBody.transform.rotation, 1);

        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, robotId);
        List<string> b3JointNames = new List<string>();
        var urdfJoints = UrdfRobot.GetComponentsInChildren<UrdfJoint>();
        b3JointIds = new List<int> ();
        
        for (int i=0; i<b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(pybullet, robotId, i, ref jointInfo);
            b3JointNames.Add(jointInfo.m_jointName);
        }
        
        foreach(var j in urdfJoints)
        {
            if (b3JointNames.Contains(j.JointName))
                b3JointIds.Add(b3JointNames.IndexOf(j.JointName));
        }



        b3IdMap = new Dictionary<GameObject, int>
        {
            { table, tableId },
            { token, tokenId },
            { upperBody, robotId }
        };

        setGravity();
        setRealTimeSimualtion(1);
    }

    // Update is called once per frame
    void Update()
    {
        UrdfRobot = GetComponent<UrdfRobot>();
        if (pybullet != IntPtr.Zero)
        {
            syncRobotJointStates(ref UrdfRobot);
            syncBodyState(token);
            
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
                
                body.transform.SetPositionAndRotation(pos, orn);
            }
        }
    }

    void syncRobotJointStates(ref UrdfRobot robot)
    {
        int robotid;
        b3IdMap.TryGetValue(upperBody, out robotid);
        IntPtr cmd_handle =
                NativeMethods.b3RequestActualStateCommandInit(pybullet, robotid);
        IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd_handle);

        EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status_handle);

        if (status_type == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
        {
            for (int i = 0; i < robot.GetComponentsInChildren<UrdfJoint>().Length; i++)
            {
                var unityJoint = robot.GetComponentsInChildren<UrdfJoint>()[i];
                if (unityJoint.JointName.Contains("TH")) continue;
                
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

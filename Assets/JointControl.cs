using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Urdf;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System;
using Valve.VR;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class JointControl : MonoBehaviour
{
    public GameObject upperBody;
    public GameObject table;
    public GameObject token;
    private UrdfRobot UrdfRobot;
    private UrdfJoint joint;
    private IntPtr pybullet;
    // Start is called before the first frame update
    void Start()
    {
        //Physics.autoSimulation = false;

        pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY2);
        //sm = (b3PhysicsClientHandle__*)pybullet;
        Debug.Log(NativeMethods.b3CanSubmitCommand(pybullet));
        IntPtr cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);


        upperBody = GameObject.Find("upper_body");
        upperBody.transform.Rotate(0, 90, 0);
        var rbs = upperBody.GetComponentsInChildren<Rigidbody>();
        foreach (var rb in rbs) {
            //rb.detectCollisions = false;
            rb.isKinematic = true;
        }
        UrdfRobot = GetComponent<UrdfRobot>();
        Debug.Log(UrdfRobot);
        foreach (UrdfJoint urdfJoint in UrdfRobot.GetComponentsInChildren<UrdfJoint>())
        {
            //Debug.Log(urdfJoint.JointName);
            if (urdfJoint.JointName == "shoulder_right_axis0")
                joint = urdfJoint;
        }

        // load URDF model

        var tablePos = table.transform.position;
        var q = table.transform.rotation;
        var tablePath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\Table\\CheckersTable.urdf";
        var tableId = loadURDF(tablePath, tablePos, q);

        var tokenPath = "C:\\Users\\roboy\\Documents\\code\\BulletUnityRoboy\\Assets\\Urdf\\token\\token.urdf";
        var tokenId = loadURDF(tokenPath, token.transform.position, token.transform.rotation);
        
        Debug.Log(tokenId.ToString());
    }

    int loadURDF(string file, Vector3 p, Quaternion q) 
    {
        var cmd = NativeMethods.b3CreateCollisionShapeCommandInit(pybullet);
        p = RosSharp.TransformExtensions.Unity2Ros(p);
        q = RosSharp.TransformExtensions.Unity2Ros(q);
        cmd = NativeMethods.b3LoadUrdfCommandInit(pybullet, file);
        NativeMethods.b3LoadUrdfCommandSetStartPosition(cmd, p.x, p.y, p.z);
        NativeMethods.b3LoadUrdfCommandSetStartOrientation(cmd, q.x, q.y, q.z, q.w);
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
        var bodyId = NativeMethods.b3GetStatusBodyIndex(status);
        return bodyId;
    }

    // Update is called once per frame
    void Update()
    {
        UrdfRobot = GetComponent<UrdfRobot>();
        if (pybullet != IntPtr.Zero)
        {
            IntPtr cmd_handle =
                NativeMethods.b3RequestActualStateCommandInit(pybullet, 0);
            IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd_handle);

            EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus)NativeMethods.b3GetStatusType(status_handle);

            if (status_type == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
            {
                for (int i=0; i<UrdfRobot.GetComponentsInChildren<UrdfJoint>().Length;i++)
                {
                    var unityJoint = UrdfRobot.GetComponentsInChildren<UrdfJoint>()[i];
                    if (unityJoint.JointName.Contains("TH")) continue;
                    b3JointSensorState state = new b3JointSensorState();
                    NativeMethods.b3GetJointState(pybullet, status_handle, i, ref state);
                    
                    if (i==0)
                    {
                        Debug.Log(unityJoint.JointName);
                        Debug.Log(state.m_jointPosition);
                        Debug.Log(unityJoint.GetPosition());
                       
                    }
                    unityJoint.UpdateJointState((float)state.m_jointPosition - unityJoint.GetPosition());
                }
                
                //Debug.Log(state.m_jointPosition);
            }
        }
        //upperBody.transform.Rotate(0, 100 * Time.deltaTime, 0);
        //joint.UpdateJointState(1 * Time.deltaTime);
        
    }

    private void OnDestroy()
    {
        if (pybullet != IntPtr.Zero)
        {
            NativeMethods.b3DisconnectSharedMemory(pybullet);
        }
    }
}

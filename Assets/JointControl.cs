using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Urdf;
using UnityEngine.UI;
using System.Runtime.InteropServices;
using System;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class JointControl : MonoBehaviour
{
    public GameObject upperBody;
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
            rb.detectCollisions = false;
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
}

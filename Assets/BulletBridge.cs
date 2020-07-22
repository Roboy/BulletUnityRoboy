using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;
using System;
using RosSharp;
using System.Threading;

[System.Runtime.InteropServices.StructLayoutAttribute(System.Runtime.InteropServices.LayoutKind.Sequential)]
public class BulletBridge : MonoBehaviour
{
    private IntPtr _pybullet;

    public IntPtr Pybullet => _pybullet;

    
    public bool isInitialized;
    private Dictionary<GameObject, int> b3IdMap;
    private float lastUpdate;
    private bool reset;

    private Thread _updateThread;

    // Start is called before the first frame update
    void Start()
    {
        _pybullet = NativeMethods.b3ConnectSharedMemory(NativeConstants.SHARED_MEMORY_KEY2);

        if (NativeMethods.b3CanSubmitCommand(_pybullet) != 1)
        {
            Debug.LogError("Cannot submit commands to pybullet server. Quitting...");
            Application.Quit();
        }

        resetSimulation();

        IntPtr cmd = NativeMethods.b3InitSyncBodyInfoCommand(_pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);

        b3IdMap = new Dictionary<GameObject, int>();

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

    public void RemoveGameObject(GameObject go)
    {
        b3IdMap.Remove(go);
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
            return _pybullet;
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


    public static void MakeKinematic(Rigidbody[] rbs)
    {
        foreach (var rb in rbs)
        {
            rb.detectCollisions = false;
            rb.isKinematic = true;
        }
    }

    public int LoadURDF(string file, Vector3 p, Quaternion q, double s, int useFixedBase = 0)
    {
        //p = p.Unity2Ros();
        //q = q.Unity2Ros();
        var cmd = NativeMethods.b3LoadUrdfCommandInit(_pybullet, file);
        NativeMethods.b3LoadUrdfCommandSetStartPosition(cmd, p.x, p.y, p.z);
        NativeMethods.b3LoadUrdfCommandSetStartOrientation(cmd, q.x, q.y, q.z, q.w);
        NativeMethods.b3LoadUrdfCommandSetGlobalScaling(cmd, s);
        NativeMethods.b3LoadUrdfCommandSetUseFixedBase(cmd, useFixedBase);

        NativeMethods.b3LoadUrdfCommandSetFlags(cmd, 264192); // = ENABLE SLEEPING + ENABLE WAKEUP

        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);
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
                NativeMethods.b3RequestActualStateCommandInit(_pybullet, bodyId);
            IntPtr status_handle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd_handle);

            EnumSharedMemoryServerStatus status_type = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(status_handle);

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

                MyPos mpos = (MyPos) Marshal.PtrToStructure(actualStateQ, typeof(MyPos));
                Vector3 pos = new Vector3((float) mpos.x, (float) mpos.y, (float) mpos.z);
                Quaternion orn = new Quaternion((float) mpos.qx, (float) mpos.qy, (float) mpos.qz, (float) mpos.qw);

                pos = RosSharp.TransformExtensions.Ros2Unity(pos);
                orn = RosSharp.TransformExtensions.Ros2Unity(orn);
                var tf = body.transform.position;
                body.transform.SetPositionAndRotation(pos, orn);
            }
        }
    }

    public void SetJointPosition(ref IntPtr cmd, int bodyId, int jointIndex, double targetPositionRad)
    {
        // b3JointInfo ji = new b3JointInfo();
        // NativeMethods.b3GetJointInfo(pybullet, bodyId, jointIndex, ref ji);
        // NativeMethods.b3JointControlSetDesiredPosition(cmd, ji.m_qIndex, targetPositionRad);
        // NativeMethods.b3JointControlSetKp(cmd, ji.m_uIndex, 0.1);
        // NativeMethods.b3JointControlSetDesiredVelocity(cmd, ji.m_uIndex, 0);
        // NativeMethods.b3JointControlSetKd(cmd, ji.m_uIndex, 1.0);
        // NativeMethods.b3JointControlSetMaximumForce(cmd, ji.m_uIndex, 100000.0);


        b3JointInfo ji = new b3JointInfo();
        NativeMethods.b3GetJointInfo(_pybullet, bodyId, jointIndex, ref ji);
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
        for (int i = start; i <= efLinkId; i++)
        {
            chain.Add(i);
            //TODO check if joint is movable
        }

        return chain;
    }

    public int b3GetLinkId(string name, int bodyId)
    {
        b3JointInfo ji = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(_pybullet, bodyId);
        for (int i = 0; i < b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(_pybullet, bodyId, i, ref ji);
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
        NativeMethods.b3GetJointInfo(_pybullet, bodyId, jointId, ref ji);
        return ji.m_jointName;
    }

    public void resetSimulation()
    {
        IntPtr cmd = NativeMethods.b3InitResetSimulationCommand(_pybullet);
        IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);
    }

    public void stepSimulation()
    {
        if (_pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitStepSimulationCommand(_pybullet);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);
        }
    }

    public void setRealTimeSimualtion(int flag)
    {
        if (_pybullet != IntPtr.Zero)
        {
            IntPtr cmd = NativeMethods.b3InitPhysicsParamCommand(_pybullet);
            NativeMethods.b3PhysicsParamSetRealTimeSimulation(cmd, flag);
            IntPtr status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);
        }
    }

    public void setGravity()
    {
        if (_pybullet != IntPtr.Zero)
        {
            IntPtr command = NativeMethods.b3InitPhysicsParamCommand(_pybullet);
            NativeMethods.b3PhysicsParamSetGravity(command, 0, 0, -9.8);
            IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, command);
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
        if (_pybullet != IntPtr.Zero)
        {
            NativeMethods.b3DisconnectSharedMemory(_pybullet);
        }
    }
}
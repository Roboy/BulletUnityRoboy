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
using System.Threading;
using JetBrains.Annotations;
using RosSharp;
using Unity.Jobs;
using UnityEngine.PlayerLoop;
using UnityEngine.SpatialTracking;
using UnityEngine.XR;
using Object = System.Object;


[RequireComponent(typeof(UrdfRobot))]
public class BulletRobot : MonoBehaviour
{
    [SerializeField] private float trackingUpdateRate = 0.02f;

    [SerializeField] private Transform cameraTF;
    [SerializeField] private bool trackIK;
    [SerializeField] private Transform leftHandTarget;
    [SerializeField] private Transform rightHandTarget;
    [SerializeField] private string urdfPath;

    [SerializeField] private List<UrdfStateInformation> robotStates;

    private IntPtr pybullet;
    private BulletBridge bb;
    private List<IkTarget> IKTargets;
    private float lastUpdate;

    private UrdfRobot urdfRobot;
    private List<int> b3JointIds;
    private List<string> b3JointNames;
    private List<int> headJointIds;
    private List<int> freeJoints;
    private List<int> wristJoints;

    private List<int> excludeFromIkJoints;

    //private int b3RobotId;
    private double camYOffset;
    private Vector3 initRobotPosition;

    private List<JointSyncInformation> jointsToSync = new List<JointSyncInformation>();


    private UrdfJoint[] robotJoints;

    /**
     * Multiple robots are spawned in the simulation to allow for seemless transition between different states (i.e. limits)
     */
    private List<int> b3RobotIds = new List<int>();

    private int _b3CurrentRobotIdIndex = 0;

    private int b3RobotId => b3RobotIds[_b3CurrentRobotIdIndex];

    /**
     * Struct to store different robot urdf paths. As of now, angle limitations cannot be modified during runtime. Therefore spawn multiple robots in simulation and sync with one.
     */
    [Serializable]
    private struct UrdfStateInformation
    {
        [SerializeField] private string _urdfPath;
        [SerializeField] private string _desc;

        public string UrdfPath => _urdfPath;

        public string Desc => _desc;
    }

    /**
     * Struct holds information about all joints that have to be synced from Bullet to Unity.
     */
    private readonly struct JointSyncInformation
    {
        private readonly int _b3JointIndex;
        private readonly UrdfJoint _urdfJoint;
        private readonly int _jointIndex;
        private readonly String _jointName;

        public UrdfJoint UrdfJoint => _urdfJoint;

        public int JointIndex => _jointIndex;

        public string JointName => _jointName;

        public int B3JointIndex => _b3JointIndex;

        public JointSyncInformation(int b3JointIndex, [NotNull] UrdfJoint urdfJoint, int jointIndex, string jointName)
        {
            this._urdfJoint = urdfJoint ? urdfJoint : throw new ArgumentNullException(nameof(urdfJoint));
            this._jointIndex = jointIndex;
            this._jointName = jointName;
            this._b3JointIndex = b3JointIndex;
        }
    }

    private struct IkTarget
    {
        public IkTarget(Transform transform, int endeffectorLinkId, List<int> kinematicChain)
        {
            this.transform = transform;
            endeffectorLinkID = endeffectorLinkId;
            this.kinematicChain = kinematicChain;
        }

        private Transform transform;
        private int endeffectorLinkID;
        private List<int> kinematicChain;

        public Transform Transform => transform;

        public int EndeffectorLinkId => endeffectorLinkID;

        public List<int> KinematicChain => kinematicChain;
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

        BulletBridge.MakeKinematic(GetComponentsInChildren<Rigidbody>());

        ResetRobotPose();

        // Load Roboy model
        urdfRobot = GetComponent<UrdfRobot>();

        // TODO: Load all Roboy models (up to robotStates.Count) => Right now huge performance issue!
        for (var i = 0; i < 1; i++)
        {
            int b3Id = bb.LoadURDF(robotStates[i].UrdfPath, transform.position + (new Vector3(i - 1 + 1, 0, 0)), transform.rotation, 1);
            Debug.Log("Loaded Robot: " + b3Id);
            b3RobotIds.Add(b3Id);
        }

        bb.AddGameObject(gameObject, b3RobotId);

        SetupRobotJoints();

        // Save all joints, as they won't change during runtime.
        this.robotJoints = urdfRobot.GetComponentsInChildren<UrdfJoint>();

        // Add inverse kinematic tracking targets
        if (trackIK)
        {
            IKTargets = new List<IkTarget>();

            if (leftHandTarget)
            {
                IKTargets.Add(new IkTarget(leftHandTarget.GetComponent<RedirectedTransform>().Transform, bb.b3GetLinkId("lh_palm", b3RobotId), bb.GetJointKinematicChain(b3RobotId, "lh_palm", "shoulder_left_link1")));
            }

            if (rightHandTarget)
            {
                IKTargets.Add(new IkTarget(rightHandTarget.GetComponent<RedirectedTransform>().Transform, bb.b3GetLinkId("rh_palm", b3RobotId), bb.GetJointKinematicChain(b3RobotId, "rh_palm", "shoulder_right_link1")));
            }
        }

        lastUpdate = Time.time * 1000;

        camYOffset = cameraTF.rotation.eulerAngles.y;

        var cmd = NativeMethods.b3InitSyncBodyInfoCommand(pybullet);
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        // Set how often FixedUpdate runs
        Time.fixedDeltaTime = trackingUpdateRate; // 0.03 = 33 Hz, 0.02 = 50 Hz
    }

    /// <summary>
    /// FixedUpdate runs every [Time.fixedDeltaTime] seconds to update joint states and track inverse kinematics targets.
    /// </summary>
    private void FixedUpdate()
    {
        SyncRobotJointStates(ref urdfRobot);

        TrackHead();
        if (trackIK)
        {
            foreach (var target in IKTargets)
                FollowObjectIk(target);
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            int numConstraints = NativeMethods.b3GetNumUserConstraints(pybullet);

            for (int i = 0; i < freeJoints.Count; i++)
            {
                b3JointInfo b3JointInfo = new b3JointInfo();
                NativeMethods.b3GetJointInfo(pybullet, b3RobotId, freeJoints[i], ref b3JointInfo);

                IntPtr cmdHandle = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId,
                    (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                NativeMethods.b3JointControlSetMaximumVelocity(cmdHandle, b3JointInfo.m_uIndex, 1.0);
                NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmdHandle);
                
                Debug.Log("_______________________________________");
            }
        }
    }

    /// <summary>
    /// Calibration of robot to vr head.
    /// </summary>
    private void ResetRobotPose()
    {
        GameObject roboyHead = GameObject.FindGameObjectWithTag("Roboy_Head");
        GameObject cameraContainer = GameObject.FindGameObjectWithTag("CameraContainer");

        //cameraContainer.transform.po = roboyHead.transform.position;
        //cameraContainer.transform.rotation = roboyHead.transform.rotation;

        //var p = new Vector3(0, 1, 0) + new Vector3(0, 0, 1) * -0.1f + cameraTF.position;
        //var p = new Vector3(0, 0, 0);
        //var cam_rotation = cameraTF.rotation;
        //Quaternion q = Quaternion.Euler(0, cam_rotation.y - 90, 0);
        //transform.SetPositionAndRotation(p, q);
    }

    private void FollowObjectIk(IkTarget target)
    {
        //var targetPosRos = target.targetTF.position.Unity2Ros();
        var targetPosRos = (target.Transform.position + (new Vector3(_b3CurrentRobotIdIndex - 1 + 1, 0, 0))).Unity2Ros(); // ToDo
        var targetOrnRos = target.Transform.rotation;
        targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
        targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
        targetOrnRos = targetOrnRos.Unity2Ros();

        double[] targetPos = {targetPosRos.x, targetPosRos.y, targetPosRos.z};
        double[] targetOrn = {targetOrnRos.x, targetOrnRos.y, targetOrnRos.z, targetOrnRos.w};

        var cmd = NativeMethods.b3CalculateInverseKinematicsCommandInit(pybullet, b3RobotId);
        NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd, target.EndeffectorLinkId,
            ref targetPos[0], ref targetOrn[0]);

        var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        int dofCount = 0;
        double[] jointTargets = new double[freeJoints.Count];
        int bodyId = b3RobotId;
        var status =
            NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount,
                ref jointTargets[0]);
        //status = NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

        cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId,
            (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
        for (int i = 0; i < jointTargets.Length; i++)
        {
            if (!target.KinematicChain.Contains(freeJoints[i])) continue;
            bb.SetJointPosition(ref cmd, b3RobotId, freeJoints[i], jointTargets[i]);
        }

        var st = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);
    }

    private void TrackHead()
    {
        var eulerAngles = cameraTF.rotation.eulerAngles;
        double pitch = eulerAngles.y - camYOffset;
        double roll = eulerAngles.x;
        double yaw = eulerAngles.z;

        //Debug.Log(BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
        IntPtr cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId, 2);
        bb.SetJointPosition(ref cmd, b3RobotId, headJointIds[0], BulletBridge.ClipAngle(roll) * Mathf.Deg2Rad);
        bb.SetJointPosition(ref cmd, b3RobotId, headJointIds[1], BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
        bb.SetJointPosition(ref cmd, b3RobotId, headJointIds[2], BulletBridge.ClipAngle(pitch) * Mathf.Deg2Rad);

        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        lastUpdate = Time.time * 1000;
    }

    private void SyncRobotJointStates(ref UrdfRobot robot)
    {
        IntPtr cmdHandle = NativeMethods.b3RequestActualStateCommandInit(pybullet, b3RobotId);

        // This takes increasingly long with more robots
        IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmdHandle);

        EnumSharedMemoryServerStatus statusType =
            (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(statusHandle);

        if (statusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
        {
            for (int i = 0; i < this.jointsToSync.Count; i++)
            {
                UrdfJoint unityJoint = this.jointsToSync[i].UrdfJoint;
                /*IEnumerator coroutine = HandleStateUpdate(unityJoint, statusHandle, this.jointsToSync[i].JointIndex);
                StartCoroutine(coroutine);*/

                b3JointSensorState state = new b3JointSensorState();
                NativeMethods.b3GetJointState(pybullet, statusHandle, this.jointsToSync[i].JointIndex, ref state);

                var diff = (float) state.m_jointPosition - unityJoint.GetPosition();

                if (unityJoint.JointName.Contains("lh_") || unityJoint.JointName.Contains("rh_"))
                {
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
                }

                unityJoint.UpdateJointState(diff);
            }
        }
        else
        {
            Debug.LogWarning("Cannot get robot state");
        }
    }

    private void SetupRobotJoints()
    {
        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(pybullet, b3RobotId);
        b3JointNames = new List<string>();
        b3JointIds = new List<int>();
        freeJoints = new List<int>();
        excludeFromIkJoints = new List<int>();

        for (int i = 0; i < b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(pybullet, b3RobotId, i, ref jointInfo);
            b3JointNames.Add(jointInfo.m_jointName);
            if (jointInfo.m_jointType == 0) //JointType.eRevoluteType)
            {
                freeJoints.Add(i);
            }

            if (jointInfo.m_jointName.Contains("rh_") || jointInfo.m_jointName.Contains("lh_") || jointInfo.m_jointName.Contains("head"))
            {
                excludeFromIkJoints.Add(i);
            }

            //if (jointInfo.m_jointName.Contains("lh"))
            //{
            //    setJointPosition(ref cmd, i, 0);
            //}
        }

        var cmd = NativeMethods.b3JointControlCommandInit2(pybullet, b3RobotId,
            (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(pybullet, cmd);

        UrdfJoint[] urdfJoints = urdfRobot.GetComponentsInChildren<UrdfJoint>();
        foreach (var j in urdfJoints)
        {
            if (b3JointNames.Contains(j.JointName))
            {
                int b3JointIndex = b3JointNames.IndexOf(j.JointName);
                jointsToSync.Add(new JointSyncInformation(b3JointIndex, j, b3JointNames.IndexOf(j.JointName), j.JointName));
            }
            else
            {
                Debug.LogWarning("pybullet doesnt know about " + j.JointName);
            }
        }

        headJointIds = new List<int>
        {
            jointsToSync.Find(item => item.JointName == "head_axis0").B3JointIndex,
            jointsToSync.Find(item => item.JointName == "head_axis2").B3JointIndex,
            jointsToSync.Find(item => item.JointName == "head_axis1").B3JointIndex,
        };
        wristJoints = new List<int>
        {
            jointsToSync.Find(item => item.JointName == "wrist_right_axis0").B3JointIndex,
            jointsToSync.Find(item => item.JointName == "wrist_right_axis1").B3JointIndex,
            jointsToSync.Find(item => item.JointName == "wrist_right_axis2").B3JointIndex,
        };
    }
}
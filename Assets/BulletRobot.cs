using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Urdf;
using System;
using System.Threading;
using Controller.Helper;
using JetBrains.Annotations;
using RosSharp;
using UnityEngine.Serialization;
using UnityEngine.XR;
using Utils;


[RequireComponent(typeof(UrdfRobot))]
public class BulletRobot : MonoBehaviour
{
    [Header("Tracking Update")] [Tooltip("How often the state gathered from Bullet is synchronized to the Unity joints.")] [SerializeField]
    private float trackingUpdateRate = 0.025f;

    [Space(10)] [Header("Limitations")] [Tooltip("How often the state is gathered from Bullet.")] [Range(0.0f, 500.0f)] [SerializeField]
    private readonly int _stateSyncUpdateRate = 15;

    [FormerlySerializedAs("trackIK")] [Space(10)] [Header("Tracking Targets")] [SerializeField]
    private bool enableIk;

    [SerializeField] private Transform leftHandTarget;
    [SerializeField] private Transform rightHandTarget;

    [Space(10)] [Header("SenseGloves")] [SerializeField]
    private bool enableGloves;

    [SerializeField] private GameObject leftHandGlove;
    [SerializeField] private GameObject rightHandGlove;

    [Space(10)] [Header("Misc")] [SerializeField]
    private Transform cameraTF;

    [FormerlySerializedAs("robotStates")] [SerializeField]
    private List<SyncedRobotInformation> syncedRobots;


    private BulletBridge _bulletBridge;

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

    private readonly List<SyncedJointsInformation> _jointsToSync = new List<SyncedJointsInformation>();

    private UrdfJoint[] robotJoints;

    // Always returns the robot, that is actively tracked
    public SyncedRobotInformation ActiveRobot => syncedRobots.Find(syncedRobot => syncedRobot.IsLoaded && syncedRobot.IsActive);

    private readonly SyncedRobotSwitchInformation _syncedRobotSwitchInformation = new SyncedRobotSwitchInformation();

    private readonly SyncedHeadInformation _syncedHeadInformation = new SyncedHeadInformation();

    private readonly List<SyncedIkTargetInformation> _syncedIkTargetInformations = new List<SyncedIkTargetInformation>();

    private readonly List<SyncedGloveInformation> _syncedGloveInformations = new List<SyncedGloveInformation>();

    public List<int> FreeJoints => freeJoints;

    public List<SyncedJointsInformation> JointsToSync => _jointsToSync;

    public SyncedRobotSwitchInformation SyncedRobotSwitchInformation => _syncedRobotSwitchInformation;

    public SyncedHeadInformation SyncedHeadInformation => _syncedHeadInformation;

    public List<SyncedIkTargetInformation> SyncedIkTargetInformations => _syncedIkTargetInformations;

    public List<SyncedGloveInformation> SyncedGloveInformations => _syncedGloveInformations;

    public List<SyncedRobotInformation> SyncedRobots => syncedRobots;

    public List<int> HeadJointIds => headJointIds;

    private float _currentTime = 0.0f;

    public Vector3 UnityPos => transform.position;

    // Start is called before the first frame update
    void Start()
    {
        _bulletBridge = GameObject.FindGameObjectWithTag("BulletConnectionController").GetComponent<BulletBridge>();
        if (!_bulletBridge.isInitialized)
        {
            Debug.LogError("no bullet bridge");
            Application.Quit();
        }

        Debug.Log("Connected " + name + " to bullet server.");

        BulletBridge.MakeKinematic(GetComponentsInChildren<Rigidbody>());

        ResetRobotPose();

        // Load Roboy model
        urdfRobot = GetComponent<UrdfRobot>();

        for (var i = 0; i < 1/*syncedRobots.Count*/; i++)
        {
            syncedRobots[i].TrackingPosition = transform.position.Unity2Ros();
            syncedRobots[i].Position = (transform.position + new Vector3(i * 1.5f, 0, 0)).Unity2Ros(); // Base position with 2 units distance
            syncedRobots[i].Rotation = transform.rotation.Unity2Ros();

            int b3RobotId = _bulletBridge.LoadURDF(syncedRobots[i].UrdfPath, syncedRobots[i].Position, syncedRobots[i].Rotation, syncedRobots[i].Scaling, 1);
            Debug.Log("Loaded Robot: " + b3RobotId);

            syncedRobots[i].IsLoaded = true;
            syncedRobots[i].B3RobotId = b3RobotId;
            syncedRobots[i].IsActive = i == 0;

            // Every robot is sleeping by default
            IntPtr sleepCommand = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
            NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand, b3RobotId, (int) DynamicsActivationState.eActivationStateSleep);
            NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, sleepCommand);
        }

        _bulletBridge.AddGameObject(gameObject, ActiveRobot.B3RobotId);

        SetupRobotJoints();

        // Save all joints, as they won't change during runtime.
        this.robotJoints = urdfRobot.GetComponentsInChildren<UrdfJoint>();

        // Add inverse kinematic tracking targets
        if (enableIk)
        {
            if (leftHandTarget)
            {
                _syncedIkTargetInformations.Add(new SyncedIkTargetInformation(leftHandTarget.transform /*leftHandTarget.GetComponent<RedirectedTransform>().Transform*/,
                    _bulletBridge.b3GetLinkId("lh_palm", ActiveRobot.B3RobotId),
                    _bulletBridge.GetJointKinematicChain(ActiveRobot.B3RobotId, "lh_palm", "shoulder_left_link1")));
            }

            if (rightHandTarget)
            {
                _syncedIkTargetInformations.Add(new SyncedIkTargetInformation(rightHandTarget.transform /*rightHandTarget.GetComponent<RedirectedTransform>().Transform*/,
                    _bulletBridge.b3GetLinkId("rh_palm", ActiveRobot.B3RobotId),
                    _bulletBridge.GetJointKinematicChain(ActiveRobot.B3RobotId, "rh_palm", "shoulder_right_link1")));
            }
        }

        if (enableGloves)
        {
            if (rightHandGlove)
            {
                _syncedGloveInformations.Add(new SyncedGloveInformation(rightHandGlove, "rh_", _jointsToSync));
            }

            if (leftHandGlove)
            {
                _syncedGloveInformations.Add(new SyncedGloveInformation(leftHandGlove, "lh_", _jointsToSync));
            }
        }

        camYOffset = cameraTF.rotation.eulerAngles.y;

        var cmd = NativeMethods.b3InitSyncBodyInfoCommand(_bulletBridge.Pybullet);
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmd);

        // Set how often FixedUpdate runs
        Time.fixedDeltaTime = trackingUpdateRate; // 0.03 = 33 Hz, 0.02 = 50 Hz

        // Wake up first robot
        IntPtr wakeUpCommand = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(wakeUpCommand, 0, (int) DynamicsActivationState.eActivationStateDisableSleeping | (int) DynamicsActivationState.eActivationStateWakeUp);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, wakeUpCommand);

        // Test other objects
        //_bulletBridge.LoadURDF("\\Urdf\\simple_box\\simple_box.urdf", new Vector3(-0.2f, 0, -2.3f), Quaternion.identity, 1);
        //_bulletBridge.LoadURDF("\\Urdf\\table2\\table.urdf", new Vector3(-0.2f, 0, -2.4f), Quaternion.identity, 1);
    }

    /// <summary>
    /// FixedUpdate runs every [Time.fixedDeltaTime] seconds to update joint states
    /// </summary>
    private void FixedUpdate()
    {
        SyncRobotJointsFromBullet();
    }

    private void Update()
    {
        _currentTime = Time.realtimeSinceStartup * 1000.0f;

        if (enableIk)
        {
            foreach (SyncedIkTargetInformation syncedIkTargetInformation in _syncedIkTargetInformations)
            {
                var targetPosRos = (syncedIkTargetInformation.Transform.position/* - (new Vector3(0, 0.02f, 0))*/).Unity2Ros(); // Correct circumstance, that tracker is slightly above hand
                var targetOrnRos = syncedIkTargetInformation.Transform.rotation;
                targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
                targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
                targetOrnRos = targetOrnRos.Unity2Ros();

                syncedIkTargetInformation.targetPos[0] = targetPosRos.x;
                syncedIkTargetInformation.targetPos[1] = targetPosRos.y;
                syncedIkTargetInformation.targetPos[2] = targetPosRos.z;

                syncedIkTargetInformation.targetOrn[0] = targetOrnRos.x;
                syncedIkTargetInformation.targetOrn[1] = targetOrnRos.y;
                syncedIkTargetInformation.targetOrn[2] = targetOrnRos.z;
                syncedIkTargetInformation.targetOrn[3] = targetOrnRos.w;
            }
        }

        if (enableGloves)
        {
            foreach (SyncedGloveInformation syncedGloveInformation in _syncedGloveInformations)
            {
                for (var i = 0; i < syncedGloveInformation.HandJointInformation.Count; i++)
                {
                    List<SyncedGloveJoint> syncedGloveJoints = syncedGloveInformation.HandJointInformation[i];
                    for (var j = 0; j < syncedGloveJoints.Count; j++)
                    {
                        //SyncedGloveJoint syncedGloveJoint = syncedGloveJoints[j];
                        Quaternion quaternion = Quaternion.identity;
                        
                        if (syncedGloveJoints[j].JointHandType == SyncedGloveJoint.HandType.Thumb)
                        {
                            // Thumb
                            switch (j)
                            {
                                case 0:
                                    quaternion = (syncedGloveInformation.HandJointInformation[i][j].JointTransform[0].rotation * syncedGloveInformation.JointCorrections[i][0]).Unity2Ros();
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(quaternion.eulerAngles.x) * Mathf.Deg2Rad + 0.6;
                                    break;
                                case 1:
                                    quaternion = (syncedGloveInformation.HandJointInformation[i][j].JointTransform[0].rotation * syncedGloveInformation.JointCorrections[i][0]).Unity2Ros();
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(quaternion.eulerAngles.y) * Mathf.Deg2Rad;
                                    break;
                                case 2:
                                    quaternion = (syncedGloveInformation.HandJointInformation[i][j].JointTransform[0].rotation * syncedGloveInformation.JointCorrections[i][0]).Unity2Ros();
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(-quaternion.eulerAngles.z) * Mathf.Deg2Rad;
                                    break;
                                default:
                                    quaternion = (syncedGloveInformation.HandJointInformation[i][j].JointTransform[1].rotation * syncedGloveInformation.JointCorrections[i][1]).Unity2Ros();
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(quaternion.eulerAngles.x) * Mathf.Deg2Rad;
                                    break;
                            }
                        }
                        else
                        {
                            quaternion = (syncedGloveInformation.HandJointInformation[i][j].JointTransform[j].rotation * syncedGloveInformation.JointCorrections[i][j]).Unity2Ros();
                            switch (j)
                            {
                                case 0:
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(-quaternion.eulerAngles.z + 100) * Mathf.Deg2Rad;
                                    break;
                                case 1:
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(-quaternion.eulerAngles.z + 60) * Mathf.Deg2Rad;
                                    break;
                                default:
                                    syncedGloveInformation.HandJointInformation[i][j].TargetJointPos = BulletBridge.ClipAngle(-quaternion.eulerAngles.z - 10) * Mathf.Deg2Rad;
                                    break;
                            }
                        }
                    }
                }
            }
        }

        _syncedHeadInformation.EulerAngles = cameraTF.rotation.eulerAngles;
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

    private void SyncRobotJointsFromBullet()
    {
        for (int i = 0; i < this._jointsToSync.Count; i++)
        {
            b3JointSensorStateWrapper b3JointSensorStateWrapper = null;

            try
            {
                if (_jointsToSync[i] == null)
                {
                    continue;
                }

                if (_jointsToSync[i].b3JointSensorStates == null)
                {
                    continue;
                }

                if (this._jointsToSync[i].b3JointSensorStates.Count == 0)
                {
                    continue;
                }


                lock (this._jointsToSync[i])
                {
                    // During a delayed scenario, there might be only actions that shall happen after the current time
                    if (this._jointsToSync[i].b3JointSensorStates.Count > 0 && this._jointsToSync[i].b3JointSensorStates.Peek() != null && this._jointsToSync[i].b3JointSensorStates.Peek().atTime > _currentTime)
                    {
                        continue;
                    }

                    // There where eventually multiple updates between the calls of this function. Therefore skip all updates, if there is a "newer" one that is still smaller than the current time
                    do
                    {
                        b3JointSensorStateWrapper = this._jointsToSync[i].b3JointSensorStates.Dequeue();
                    } while (this._jointsToSync[i].b3JointSensorStates.Count > 0 && this._jointsToSync[i].b3JointSensorStates.Peek() != null && this._jointsToSync[i].b3JointSensorStates.Peek().atTime <= _currentTime);
                }
            }
            catch (Exception e)
            {
                continue;
            }


            if (b3JointSensorStateWrapper == null)
            {
                continue;
            }

            UrdfJoint unityJoint = this._jointsToSync[i].UrdfJoint;
            var diff = (float) b3JointSensorStateWrapper.b3JointSensorState.m_jointPosition - unityJoint.GetPosition();
            if (unityJoint.JointName.Contains("lh_") || unityJoint.JointName.Contains("rh_"))
            {
                // ToDo: SenseGloves Stuff
                var pos = new Vector3((float) b3JointSensorStateWrapper.b3LinkState.m_worldPosition[0], (float) b3JointSensorStateWrapper.b3LinkState.m_worldPosition[1],
                    (float) b3JointSensorStateWrapper.b3LinkState.m_worldPosition[2]);
                var orn = new Quaternion((float) b3JointSensorStateWrapper.b3LinkState.m_worldOrientation[0], (float) b3JointSensorStateWrapper.b3LinkState.m_worldOrientation[1],
                    (float) b3JointSensorStateWrapper.b3LinkState.m_worldOrientation[2], (float) b3JointSensorStateWrapper.b3LinkState.m_worldOrientation[3]);

                pos = pos.Ros2Unity();
                orn = orn.Ros2Unity();

                var tf = this._jointsToSync[i].UrdfLink.transform.position;
                this._jointsToSync[i].UrdfLink.transform.SetPositionAndRotation(pos, orn);

                continue;
            }

            unityJoint.UpdateJointState(diff);
        }
    }


    private void SetupRobotJoints()
    {
        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(_bulletBridge.Pybullet, ActiveRobot.B3RobotId);
        b3JointNames = new List<string>();
        b3JointIds = new List<int>();
        List<int> b3LinkIds = new List<int>();
        freeJoints = new List<int>();
        excludeFromIkJoints = new List<int>();

        for (int i = 0; i < b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(_bulletBridge.Pybullet, ActiveRobot.B3RobotId, i, ref jointInfo);
            b3JointNames.Add(jointInfo.m_jointName);
            b3LinkIds.Add(i);
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

        var cmd = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmd);

        UrdfJoint[] urdfJoints = urdfRobot.GetComponentsInChildren<UrdfJoint>();
        foreach (var j in urdfJoints)
        {
            if (b3JointNames.Contains(j.JointName))
            {
                UrdfLink urdfLink = j.gameObject.GetComponent<UrdfLink>();

                int b3JointIndex = b3JointNames.IndexOf(j.JointName);
                int b3LinkIndex = b3LinkIds[b3JointNames.IndexOf(j.JointName)];
                _jointsToSync.Add(new SyncedJointsInformation(b3JointIndex, j, b3JointNames.IndexOf(j.JointName), j.JointName, b3LinkIndex, urdfLink));
            }
            else
            {
                Debug.LogWarning("pybullet doesnt know about " + j.JointName);
            }
        }

        headJointIds = new List<int>
        {
            _jointsToSync.Find(item => item.JointName == "head_axis0").B3JointIndex,
            _jointsToSync.Find(item => item.JointName == "head_axis2").B3JointIndex,
            _jointsToSync.Find(item => item.JointName == "head_axis1").B3JointIndex,
        };
        wristJoints = new List<int>
        {
            _jointsToSync.Find(item => item.JointName == "wrist_right_axis0").B3JointIndex,
            _jointsToSync.Find(item => item.JointName == "wrist_right_axis1").B3JointIndex,
            _jointsToSync.Find(item => item.JointName == "wrist_right_axis2").B3JointIndex,
        };
    }

    #region Deprecated Functions

    [System.Obsolete("Deprecated. Replaced by Threads.")]
    private void SyncRobotJointStates(ref UrdfRobot robot)
    {
        IntPtr cmdHandle = NativeMethods.b3RequestActualStateCommandInit(_bulletBridge.Pybullet, ActiveRobot.B3RobotId);
        cmdHandle = NativeMethods.b3RequestActualStateCommandInit2(cmdHandle, ActiveRobot.B3RobotId);

        IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmdHandle);

        EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(statusHandle);

        if (statusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
        {
            for (int i = 0; i < this._jointsToSync.Count; i++)
            {
                UrdfJoint unityJoint = this._jointsToSync[i].UrdfJoint;

                b3JointSensorState state = new b3JointSensorState();
                NativeMethods.b3GetJointState(_bulletBridge.Pybullet, statusHandle, this._jointsToSync[i].JointIndex, ref state);

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

    [System.Obsolete("Deprecated. Replaced by Threads.")]
    private void FollowObjectIk(SyncedIkTargetInformation targetData)
    {
        //var targetPosRos = target.targetTF.position.Unity2Ros();
        var targetPosRos = (targetData.Transform.position + (new Vector3(ActiveRobot.B3RobotId - 1 + 1, 0, 0))).Unity2Ros(); // ToDo
        var targetOrnRos = targetData.Transform.rotation;
        targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
        targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
        targetOrnRos = targetOrnRos.Unity2Ros();

        double[] targetPos = {targetPosRos.x, targetPosRos.y, targetPosRos.z};
        double[] targetOrn = {targetOrnRos.x, targetOrnRos.y, targetOrnRos.z, targetOrnRos.w};

        int dofCount = 0;
        double[] jointTargets = new double[freeJoints.Count];
        int bodyId = ActiveRobot.B3RobotId;

        var cmd = NativeMethods.b3CalculateInverseKinematicsCommandInit(_bulletBridge.Pybullet, ActiveRobot.B3RobotId);
        NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd, targetData.EndeffectorLinkId, ref targetPos[0], ref targetOrn[0]);

        var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmd);

        NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

        foreach (SyncedRobotInformation syncedRobotInformation in syncedRobots.FindAll(item => item.IsLoaded && item.IsActive))
        {
            cmd = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, syncedRobotInformation.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
            for (int i = 0; i < jointTargets.Length; i++)
            {
                if (!targetData.KinematicChain.Contains(freeJoints[i]))
                {
                    continue;
                }

                _bulletBridge.SetJointPosition(ref cmd, syncedRobotInformation.B3RobotId, freeJoints[i], jointTargets[i]);
            }

            NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmd);
        }
    }

    [System.Obsolete("Deprecated. Replaced by Threads.")]
    private void TrackHead()
    {
        var eulerAngles = cameraTF.rotation.eulerAngles;
        double pitch = eulerAngles.y - camYOffset;
        double roll = eulerAngles.x;
        double yaw = eulerAngles.z;

        //Debug.Log(BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
        IntPtr cmd = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, ActiveRobot.B3RobotId, 2);
        _bulletBridge.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[0], BulletBridge.ClipAngle(roll) * Mathf.Deg2Rad);
        _bulletBridge.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[1], BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
        _bulletBridge.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[2], BulletBridge.ClipAngle(pitch) * Mathf.Deg2Rad);

        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmd);
    }

    /**
     * Switches, which robot is synced. Always chooses the next robot in [syncedRobots].
     */
    [System.Obsolete("Deprecated. Replaced by Threads.")]
    private IEnumerator SwitchRobotControl()
    {
        int currentRobotArrIndex = syncedRobots.IndexOf(ActiveRobot);
        if (currentRobotArrIndex + 1 >= syncedRobots.Count)
        {
            yield return 0;
        }

        SyncedRobotInformation prevSyncedRobot = syncedRobots[currentRobotArrIndex];
        SyncedRobotInformation nextSyncedRobot = syncedRobots[currentRobotArrIndex + 1];

        Debug.Log("Switchting from " + prevSyncedRobot.B3RobotId + " to " + nextSyncedRobot.B3RobotId);

        IntPtr wakeUpCommand = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(wakeUpCommand, nextSyncedRobot.B3RobotId,
            (int) DynamicsActivationState.eActivationStateDisableSleeping | (int) DynamicsActivationState.eActivationStateWakeUp);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, wakeUpCommand);
        nextSyncedRobot.IsActive = true;

        // ToDo: Work out best way. 0.03 should give enough room for initial synchronisation. Still a bit laggy.
        yield return new WaitForSeconds(0.03f);

        prevSyncedRobot.IsActive = false;
        IntPtr sleepCommand = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand, prevSyncedRobot.B3RobotId, (int) DynamicsActivationState.eActivationStateEnableSleeping | (int) DynamicsActivationState.eActivationStateSleep);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, sleepCommand);

        yield return new WaitForSeconds(0.25f); // ToDo: May be possible to chose even smaller. Don't know though why it is necessary

        IntPtr sleepCommand2 = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand2, prevSyncedRobot.B3RobotId, (int) DynamicsActivationState.eActivationStateSleep);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, sleepCommand2);

        yield return 1;
    }

    #endregion
}
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosSharp.Urdf;
using System;
using System.Threading;
using JetBrains.Annotations;
using RosSharp;
using UnityEngine.Serialization;


[RequireComponent(typeof(UrdfRobot))]
public class BulletRobot : MonoBehaviour
{
    [Header("Tracking Update")] [Tooltip("How often the state gathered from Bullet is synchronized to the Unity joints.")] [SerializeField]
    private float trackingUpdateRate = 0.02f;

    [Space(10)] [Header("Limitations")] [Tooltip("How often the state is gathered from Bullet.")] [Range(0.0f, 500.0f)] [SerializeField]
    private readonly int _stateSyncUpdateRate = 15;

    [Space(10)] [Header("Limitations")] [Range(0.0f, 500.0f)] [SerializeField]
    private float trackingDelay = 0.0f;

    [FormerlySerializedAs("trackIK")] [Space(10)] [Header("Tracking Targets")] [SerializeField]
    private bool enableIk;

    [SerializeField] private Transform leftHandTarget;
    [SerializeField] private Transform rightHandTarget;

    [Space(10)] [Header("Misc")] [SerializeField]
    private Transform cameraTF;

    [FormerlySerializedAs("robotStates")] [SerializeField]
    private List<SyncedRobotInformation> syncedRobots;

    
    private IntPtr _pybullet;
    private BulletBridge bb;

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


    private SyncedRobotInformation ActiveRobot => syncedRobots.Find(syncedRobot => syncedRobot.IsLoaded && syncedRobot.IsActive);


    /**
     * Struct to perform averaged profiling.
     * Usage:       float tmpTimeC = Time.realtimeSinceStartup;
                    measurePointC.AccumulatedTime += Time.realtimeSinceStartup - tmpTimeC;
                    measurePointC.Count += 1;
     */
    private struct MeasureStruct
    {
        private float _accumulatedTime;
        private float _count;

        public MeasureStruct(float accumulatedTime, float count)
        {
            this._accumulatedTime = accumulatedTime;
            this._count = count;
        }

        public float AccumulatedTime
        {
            get => _accumulatedTime;
            set => _accumulatedTime = value;
        }

        public float Count
        {
            get => _count;
            set => _count = value;
        }

        public String PrintResult()
        {
            return "Time: " + _accumulatedTime / _count;
        }
    }

    /**
     * Class to store different robot urdf paths. As of now, angle limitations cannot be modified during runtime. Therefore spawn multiple robots in simulation and sync with one.
     */
    [Serializable]
    private class SyncedRobotInformation
    {
        [FormerlySerializedAs("_urdfPath")] [SerializeField]
        private string urdfPath;

        [FormerlySerializedAs("_desc")] [SerializeField]
        private string desc;

        private int _b3RobotId;
        private bool _isActive;
        private bool _isLoaded;
        private Vector3 _position;
        private Quaternion _rotation;
        private Vector3 _trackingPosition; // Position, if this robot is being tracked (to make it accessible from a thread)

        public string UrdfPath => urdfPath;

        public string Desc => desc;

        public bool IsLoaded
        {
            get => _isLoaded;
            set => _isLoaded = value;
        }

        public int B3RobotId
        {
            get => _b3RobotId;
            set => _b3RobotId = value;
        }

        public bool IsActive
        {
            get => _isActive;
            set => _isActive = value;
        }

        public Vector3 Position
        {
            get => _position;
            set => _position = value;
        }

        public Quaternion Rotation
        {
            get => _rotation;
            set => _rotation = value;
        }

        public Vector3 TrackingPosition
        {
            get => _trackingPosition;
            set => _trackingPosition = value;
        }
    }

    /**
     * Class holds information about all joints that have to be synced from Bullet to Unity.
     */
    private class JointSyncInformation
    {
        private readonly int _b3JointIndex;
        private readonly UrdfJoint _urdfJoint;
        private readonly int _jointIndex;
        private readonly String _jointName;

        public Queue<b3JointSensorStateWrapper> b3JointSensorStates = new Queue<b3JointSensorStateWrapper>();

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

    private class b3JointSensorStateWrapper
    {
        public b3JointSensorState b3JointSensorState;
        public float currentTime;

        public b3JointSensorStateWrapper(b3JointSensorState b3JointSensorState, float currentTime)
        {
            this.b3JointSensorState = b3JointSensorState;
            this.currentTime = currentTime;
        }
    }

    /**
     * Class holds information for the event when a switch between robots occurs.
     */
    private class SwitchRobotData
    {
        private bool _switchRobotFlag;
        private int _waitCounterA;
        private int _waitCounterB;
        private SyncedRobotInformation _prevSyncedRobot;
        private SyncedRobotInformation _nextSyncedRobot;

        public SwitchRobotData()
        {
            _switchRobotFlag = false;
            _waitCounterA = -1;
            _waitCounterB = -1;
            _prevSyncedRobot = null;
            _nextSyncedRobot = null;
        }

        public bool SwitchRobotFlag
        {
            get => _switchRobotFlag;
            set => _switchRobotFlag = value;
        }

        public int WaitCounterA
        {
            get => _waitCounterA;
            set => _waitCounterA = value;
        }

        public int WaitCounterB
        {
            get => _waitCounterB;
            set => _waitCounterB = value;
        }

        public SyncedRobotInformation PrevSyncedRobot
        {
            get => _prevSyncedRobot;
            set => _prevSyncedRobot = value;
        }

        public SyncedRobotInformation NextSyncedRobot
        {
            get => _nextSyncedRobot;
            set => _nextSyncedRobot = value;
        }
    }

    private readonly SwitchRobotData _switchRobotData = new SwitchRobotData();

    /**
     * Class holds information about head movement.
     */
    private class HeadData
    {
        private double _camYOffset;
        private Vector3 _eulerAngles;

        public double CamYOffset
        {
            set => _camYOffset = value;
        }

        public Vector3 EulerAngles
        {
            set => _eulerAngles = value;
        }

        public double Pitch => _eulerAngles.y - _camYOffset;
        public double Roll => _eulerAngles.x;
        public double Yaw => _eulerAngles.z;
    }

    private readonly HeadData _headData = new HeadData();

    /**
     * Class holds information for joints that are tracked (used for IK calculation)
     */
    private class IkTargetData
    {
        public IkTargetData(Transform transform, int endeffectorLinkId, List<int> kinematicChain)
        {
            this._transform = transform;
            this._endeffectorLinkId = endeffectorLinkId;
            this._kinematicChain = kinematicChain;
        }

        private readonly Transform _transform;
        private readonly int _endeffectorLinkId;
        private readonly List<int> _kinematicChain;

        public double[] targetPos = {0, 0, 0};
        public double[] targetOrn = {0, 0, 0, 0};

        public Transform Transform => _transform;

        public int EndeffectorLinkId => _endeffectorLinkId;

        public List<int> KinematicChain => _kinematicChain;
    }

    private readonly List<IkTargetData> _ikTargetData = new List<IkTargetData>();

    private Thread _stateSyncThread;

    private float _currentTime = 0.0f;

    // Start is called before the first frame update
    void Start()
    {
        bb = GameObject.Find("BulletBridge").GetComponent<BulletBridge>();
        if (!bb.isInitialized)
        {
            Debug.LogError("no bullet bridge");
            Application.Quit();
        }

        _pybullet = bb.GetPhysicsServerPtr();
        Debug.Log("Connected " + name + " to bullet server.");

        BulletBridge.MakeKinematic(GetComponentsInChildren<Rigidbody>());

        ResetRobotPose();

        // Load Roboy model
        urdfRobot = GetComponent<UrdfRobot>();

        for (var i = 0; i < syncedRobots.Count; i++)
        {
            syncedRobots[i].TrackingPosition = transform.position.Unity2Ros();
            syncedRobots[i].Position = (transform.position + new Vector3(i * 1.5f, 0, 0)).Unity2Ros(); // Base position with 2 units distance
            syncedRobots[i].Rotation = transform.rotation.Unity2Ros();

            int b3RobotId = bb.LoadURDF(syncedRobots[i].UrdfPath, syncedRobots[i].Position, syncedRobots[i].Rotation, 1);
            Debug.Log("Loaded Robot: " + b3RobotId);

            syncedRobots[i].IsLoaded = true;
            syncedRobots[i].B3RobotId = b3RobotId;
            syncedRobots[i].IsActive = i == 0;

            // Every robot is sleeping by default
            IntPtr sleepCommand = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
            NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand, b3RobotId, (int) DynamicsActivationState.eActivationStateSleep);
            NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, sleepCommand);
        }

        bb.AddGameObject(gameObject, ActiveRobot.B3RobotId);

        SetupRobotJoints();

        // Save all joints, as they won't change during runtime.
        this.robotJoints = urdfRobot.GetComponentsInChildren<UrdfJoint>();

        // Add inverse kinematic tracking targets
        if (enableIk)
        {
            if (leftHandTarget)
            {
                _ikTargetData.Add(new IkTargetData(leftHandTarget.transform /*leftHandTarget.GetComponent<RedirectedTransform>().Transform*/, bb.b3GetLinkId("lh_palm", ActiveRobot.B3RobotId),
                    bb.GetJointKinematicChain(ActiveRobot.B3RobotId, "lh_palm", "shoulder_left_link1")));
            }

            if (rightHandTarget)
            {
                _ikTargetData.Add(new IkTargetData(rightHandTarget.transform /*rightHandTarget.GetComponent<RedirectedTransform>().Transform*/, bb.b3GetLinkId("rh_palm", ActiveRobot.B3RobotId),
                    bb.GetJointKinematicChain(ActiveRobot.B3RobotId, "rh_palm", "shoulder_right_link1")));
            }
        }

        camYOffset = cameraTF.rotation.eulerAngles.y;

        var cmd = NativeMethods.b3InitSyncBodyInfoCommand(_pybullet);
        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);

        // Set how often FixedUpdate runs
        Time.fixedDeltaTime = trackingUpdateRate; // 0.03 = 33 Hz, 0.02 = 50 Hz

        // Wake up first robot
        IntPtr wakeUpCommand = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(wakeUpCommand, 0, (int) DynamicsActivationState.eActivationStateDisableSleeping | (int) DynamicsActivationState.eActivationStateWakeUp);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, wakeUpCommand);

        // Start state synchronisation
        _stateSyncThread = new Thread(StateSyncThread);
        _stateSyncThread.Start();
    }

    /// <summary>
    /// FixedUpdate runs every [Time.fixedDeltaTime] seconds to update joint states
    /// </summary>
    private void FixedUpdate()
    {
        SyncRobotJointsFromBullet();

        //TrackHead();
        // if (trackIK)
        // {
        //     foreach (var target in IKTargets)
        //         FollowObjectIk(target);
        // }
    }

    private void OnApplicationQuit()
    {
        _stateSyncThread.Abort();
    }

    private bool _moveRobotFlag = false;
    private void Update()
    {
        _currentTime = Time.realtimeSinceStartup * 1000.0f;

        if (enableIk)
        {
            foreach (IkTargetData ikTarget in _ikTargetData)
            {
                //var targetPosRos = (ikTarget.Transform.position + (new Vector3(ActiveRobot.B3RobotId - 1 + 1, 0, 0))).Unity2Ros(); // ToDo
                var targetPosRos = (ikTarget.Transform.position /* + (new Vector3(ActiveRobot.B3RobotId - 1 + 1, 0, 0))*/).Unity2Ros(); // ToDo
                var targetOrnRos = ikTarget.Transform.rotation;
                targetOrnRos *= Quaternion.AngleAxis(90, new Vector3(0, 1, 0));
                targetOrnRos *= Quaternion.AngleAxis(180, new Vector3(1, 0, 0));
                targetOrnRos = targetOrnRos.Unity2Ros();

                ikTarget.targetPos[0] = targetPosRos.x;
                ikTarget.targetPos[1] = targetPosRos.y;
                ikTarget.targetPos[2] = targetPosRos.z;

                ikTarget.targetOrn[0] = targetOrnRos.x;
                ikTarget.targetOrn[1] = targetOrnRos.y;
                ikTarget.targetOrn[2] = targetOrnRos.z;
                ikTarget.targetOrn[3] = targetOrnRos.w;
            }
        }

        _headData.EulerAngles = cameraTF.rotation.eulerAngles;

        if (Input.GetKeyDown(KeyCode.A))
        {
            _moveRobotFlag = true;
        }
        
        if (Input.GetKeyDown(KeyCode.Return))
        {
            _switchRobotData.SwitchRobotFlag = true;
        }

        if (Input.GetKeyDown(KeyCode.Space))
        {
            for (int i = 0; i < freeJoints.Count; i++)
            {
                b3JointInfo b3JointInfo = new b3JointInfo();
                NativeMethods.b3GetJointInfo(_pybullet, ActiveRobot.B3RobotId, freeJoints[i], ref b3JointInfo);

                IntPtr cmdHandle = NativeMethods.b3JointControlCommandInit2(_pybullet, ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                NativeMethods.b3JointControlSetMaximumVelocity(cmdHandle, b3JointInfo.m_uIndex, 1.0);
                NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmdHandle);
            }
        }

        if (Input.GetKeyDown(KeyCode.A))
        {
            //NativeMethods.b3ChangeDynamicsInfoSetAngularDamping(cmdHandle, ActiveRobot.B3RobotId, 0.01);

            // Reduces velocity, but just clamps during calculation. Yields weird movements to solve IK.
            //NativeMethods.b3ChangeDynamicsInfoSetMaxJointVelocity(cmdHandle, ActiveRobot.B3RobotId, 5);


            for (int i = 0; i < freeJoints.Count; i++)
            {
                IntPtr cmdHandle = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
                b3JointInfo b3JointInfo = new b3JointInfo();
                NativeMethods.b3GetJointInfo(_pybullet, ActiveRobot.B3RobotId, freeJoints[i], ref b3JointInfo);

                //NativeMethods.b3ChangeDynamicsInfoSetAngularDamping(cmdHandle, ActiveRobot.B3RobotId, 0.1);
                NativeMethods.b3ChangeDynamicsInfoSetJointDamping(cmdHandle, ActiveRobot.B3RobotId, b3JointInfo.m_uIndex, 0.1);
                NativeMethods.b3ChangeDynamicsInfoSetContactStiffnessAndDamping(cmdHandle, ActiveRobot.B3RobotId, b3JointInfo.m_uIndex, 100, 0.2);
                NativeMethods.b3ChangeDynamicsInfoSetLinearDamping(cmdHandle, ActiveRobot.B3RobotId, 0.8);
                NativeMethods.b3ChangeDynamicsInfoSetAngularDamping(cmdHandle, ActiveRobot.B3RobotId, 0.8);
                NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmdHandle);
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


    /**
     * This function is executed on a different thread. It handles all communication with Bullet server.
     */
    private void StateSyncThread()
    {
        while (_stateSyncThread.IsAlive)
        {
            // IK
            foreach (IkTargetData ikTarget in _ikTargetData)
            {
                var inverseKinematicsCommand = NativeMethods.b3CalculateInverseKinematicsCommandInit(_pybullet, ActiveRobot.B3RobotId);
                NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(inverseKinematicsCommand, ikTarget.EndeffectorLinkId, ref ikTarget.targetPos[0], ref ikTarget.targetOrn[0]);
                var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, inverseKinematicsCommand);

                int dofCount = 0;
                double[] jointTargets = new double[freeJoints.Count];
                int bodyId = ActiveRobot.B3RobotId;

                NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

                foreach (SyncedRobotInformation syncedRobotInformation in syncedRobots.FindAll(item => item.IsLoaded && item.IsActive))
                {
                    inverseKinematicsCommand = NativeMethods.b3JointControlCommandInit2(_pybullet, syncedRobotInformation.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                    for (int i = 0; i < jointTargets.Length; i++)
                    {
                        if (!ikTarget.KinematicChain.Contains(freeJoints[i]))
                        {
                            continue;
                        }

                        bb.SetJointPosition(ref inverseKinematicsCommand, syncedRobotInformation.B3RobotId, freeJoints[i], jointTargets[i]);
                    }

                    NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, inverseKinematicsCommand);
                }
            }

            // Update State
            IntPtr actualStateCommand = NativeMethods.b3RequestActualStateCommandInit(_pybullet, ActiveRobot.B3RobotId);
            actualStateCommand = NativeMethods.b3RequestActualStateCommandInit2(actualStateCommand, ActiveRobot.B3RobotId);

            IntPtr jointStatusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, actualStateCommand);
            EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(jointStatusHandle);
            if (statusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
            {
                for (int i = 0; i < this.jointsToSync.Count; i++)
                {
                    b3JointSensorState b3JointSensorState = new b3JointSensorState();
                    NativeMethods.b3GetJointState(_pybullet, jointStatusHandle, this.jointsToSync[i].JointIndex, ref b3JointSensorState);
                    b3JointSensorStateWrapper b3JointSensorStateWrapper = new b3JointSensorStateWrapper(b3JointSensorState, _currentTime + trackingDelay);
                    this.jointsToSync[i].b3JointSensorStates.Enqueue(b3JointSensorStateWrapper);

                    //NativeMethods.b3GetJointState(pybullet, jointStatusHandle, this.jointsToSync[i].JointIndex, ref jointsToSync[i].b3JointSensorState);
                }
            }

            // Track Head
            IntPtr cmd = NativeMethods.b3JointControlCommandInit2(_pybullet, ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
            bb.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[0], BulletBridge.ClipAngle(_headData.Roll) * Mathf.Deg2Rad);
            bb.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[1], BulletBridge.ClipAngle(_headData.Yaw) * Mathf.Deg2Rad);
            bb.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[2], BulletBridge.ClipAngle(_headData.Pitch) * Mathf.Deg2Rad);
            NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);

            // Switch Robot Control
            if (_switchRobotData.SwitchRobotFlag || _switchRobotData.WaitCounterA != 0 || _switchRobotData.WaitCounterB != 0)
            {
                if (_switchRobotData.SwitchRobotFlag)
                {
                    _switchRobotData.SwitchRobotFlag = false;
                    _switchRobotData.WaitCounterA = 0;
                    _switchRobotData.WaitCounterB = 0;

                    int currentRobotArrIndex = syncedRobots.IndexOf(ActiveRobot);
                    if (currentRobotArrIndex + 1 >= syncedRobots.Count)
                    {
                        continue;
                    }

                    _switchRobotData.PrevSyncedRobot = syncedRobots[currentRobotArrIndex];
                    _switchRobotData.NextSyncedRobot = syncedRobots[currentRobotArrIndex + 1];

                    Debug.Log("Switchting from " + _switchRobotData.PrevSyncedRobot.B3RobotId + " to " + _switchRobotData.NextSyncedRobot.B3RobotId);

                    IntPtr wakeUpCommand = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
                    NativeMethods.b3ChangeDynamicsInfoSetActivationState(wakeUpCommand, _switchRobotData.NextSyncedRobot.B3RobotId,
                        (int) DynamicsActivationState.eActivationStateDisableSleeping | (int) DynamicsActivationState.eActivationStateWakeUp);
                    NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, wakeUpCommand);
                    _switchRobotData.NextSyncedRobot.IsActive = true;
                }

                // Happens after 15*[_stateSyncUpdateRate], to give time for the simulation to update state
                if (_switchRobotData.WaitCounterA == 15)
                {
                    _switchRobotData.WaitCounterA = -1;
                    
                    // The next robot moves to the position of the old robot
                    _switchRobotData.NextSyncedRobot.Position = _switchRobotData.PrevSyncedRobot.Position;

                    // Old robot moves to an (arbitray) different position, this robot is not used anymore
                    _switchRobotData.PrevSyncedRobot.Position = new Vector3(_switchRobotData.PrevSyncedRobot.Position.x - ((_switchRobotData.PrevSyncedRobot.B3RobotId + 1) * 1.5f), _switchRobotData.PrevSyncedRobot.Position.y, _switchRobotData.PrevSyncedRobot.Position.z);
                    
                    // Move old robot to its new position
                    IntPtr movePrevRobotCmdHandle = NativeMethods.b3CreatePoseCommandInit(_pybullet, _switchRobotData.PrevSyncedRobot.B3RobotId);
                    NativeMethods.b3CreatePoseCommandSetBasePosition(movePrevRobotCmdHandle, _switchRobotData.PrevSyncedRobot.Position.x, _switchRobotData.PrevSyncedRobot.Position.y, _switchRobotData.PrevSyncedRobot.Position.z);
                    NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, movePrevRobotCmdHandle);

                    // Move new robot to its new position
                    IntPtr moveNextRobotCmdHandle = NativeMethods.b3CreatePoseCommandInit(_pybullet, _switchRobotData.NextSyncedRobot.B3RobotId);
                    NativeMethods.b3CreatePoseCommandSetBasePosition(moveNextRobotCmdHandle, _switchRobotData.NextSyncedRobot.Position.x, _switchRobotData.NextSyncedRobot.Position.y, _switchRobotData.NextSyncedRobot.Position.z + 0.625); // ToDo: Why is the Z value not right?
                    NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, moveNextRobotCmdHandle);
                    
                    _switchRobotData.PrevSyncedRobot.IsActive = false;
                    IntPtr sleepCommand = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
                    NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand, _switchRobotData.PrevSyncedRobot.B3RobotId,
                        (int) DynamicsActivationState.eActivationStateEnableSleeping | (int) DynamicsActivationState.eActivationStateSleep);
                    NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, sleepCommand);
                    continue;
                }

                if (_switchRobotData.WaitCounterA != -1)
                {
                    _switchRobotData.WaitCounterA++;
                }

                // Happens after 30*[_stateSyncUpdateRate]
                // Not sure, why necessary. But if the state command is not set again after a bit of time, performance decreases.
                if (_switchRobotData.WaitCounterB == 30)
                {
                    _switchRobotData.WaitCounterB = -1;

                    IntPtr sleepCommand2 = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
                    NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand2, _switchRobotData.PrevSyncedRobot.B3RobotId, (int) DynamicsActivationState.eActivationStateSleep);
                    NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, sleepCommand2);

                    _switchRobotData.PrevSyncedRobot = null;
                    _switchRobotData.NextSyncedRobot = null;
                    continue;
                }

                if (_switchRobotData.WaitCounterB != -1)
                {
                    _switchRobotData.WaitCounterB++;
                }
            }

            if (_moveRobotFlag == true)
            {
                _moveRobotFlag = false;
                IntPtr cmdHandle = NativeMethods.b3CreatePoseCommandInit(_pybullet, ActiveRobot.B3RobotId);
                NativeMethods.b3CreatePoseCommandSetBasePosition(cmdHandle, -1, 0, 0);
                NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmdHandle);
            }

            Thread.Sleep(_stateSyncUpdateRate);
        }
    }

    private void SyncRobotJointsFromBullet()
    {
        for (int i = 0; i < this.jointsToSync.Count; i++)
        {
            if (this.jointsToSync[i].b3JointSensorStates.Count == 0)
            {
                continue;
            }

            // During a delayed scenario, there might be only actions that shall happen after the current time
            if (this.jointsToSync[i].b3JointSensorStates.Peek().currentTime > _currentTime)
            {
                continue;
            }

            b3JointSensorStateWrapper b3JointSensorStateWrapper = this.jointsToSync[i].b3JointSensorStates.Dequeue();
            // There where eventually multiple updates between the calls of this function. Therefore skip all updates, if there is a "newer" one that is still smaller than the current time
            while (this.jointsToSync[i].b3JointSensorStates.Count > 0 && this.jointsToSync[i].b3JointSensorStates.Peek().currentTime <= _currentTime)
            {
                b3JointSensorStateWrapper = this.jointsToSync[i].b3JointSensorStates.Dequeue();
            }

            UrdfJoint unityJoint = this.jointsToSync[i].UrdfJoint;
            var diff = (float) b3JointSensorStateWrapper.b3JointSensorState.m_jointPosition - unityJoint.GetPosition();
            if (unityJoint.JointName.Contains("lh_") || unityJoint.JointName.Contains("rh_"))
            {
                // ToDo: SenseGloves Stuff
                continue;
            }

            unityJoint.UpdateJointState(diff);
        }
    }


    private void SetupRobotJoints()
    {
        b3JointInfo jointInfo = new b3JointInfo();
        var b3JointsNum = NativeMethods.b3GetNumJoints(_pybullet, ActiveRobot.B3RobotId);
        b3JointNames = new List<string>();
        b3JointIds = new List<int>();
        freeJoints = new List<int>();
        excludeFromIkJoints = new List<int>();

        for (int i = 0; i < b3JointsNum; i++)
        {
            NativeMethods.b3GetJointInfo(_pybullet, ActiveRobot.B3RobotId, i, ref jointInfo);
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

        var cmd = NativeMethods.b3JointControlCommandInit2(_pybullet, ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);

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

    #region Deprecated Functions

    [System.Obsolete("Deprecated. Replaced by Threads.")]
    private void SyncRobotJointStates(ref UrdfRobot robot)
    {
        IntPtr cmdHandle = NativeMethods.b3RequestActualStateCommandInit(_pybullet, ActiveRobot.B3RobotId);
        cmdHandle = NativeMethods.b3RequestActualStateCommandInit2(cmdHandle, ActiveRobot.B3RobotId);

        IntPtr statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmdHandle);

        EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(statusHandle);

        if (statusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
        {
            for (int i = 0; i < this.jointsToSync.Count; i++)
            {
                UrdfJoint unityJoint = this.jointsToSync[i].UrdfJoint;

                b3JointSensorState state = new b3JointSensorState();
                NativeMethods.b3GetJointState(_pybullet, statusHandle, this.jointsToSync[i].JointIndex, ref state);

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
    private void FollowObjectIk(IkTargetData targetData)
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

        var cmd = NativeMethods.b3CalculateInverseKinematicsCommandInit(_pybullet, ActiveRobot.B3RobotId);
        NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(cmd, targetData.EndeffectorLinkId, ref targetPos[0], ref targetOrn[0]);

        var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);

        NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

        foreach (SyncedRobotInformation syncedRobotInformation in syncedRobots.FindAll(item => item.IsLoaded && item.IsActive))
        {
            cmd = NativeMethods.b3JointControlCommandInit2(_pybullet, syncedRobotInformation.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
            for (int i = 0; i < jointTargets.Length; i++)
            {
                if (!targetData.KinematicChain.Contains(freeJoints[i]))
                {
                    continue;
                }

                bb.SetJointPosition(ref cmd, syncedRobotInformation.B3RobotId, freeJoints[i], jointTargets[i]);
            }

            NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);
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
        IntPtr cmd = NativeMethods.b3JointControlCommandInit2(_pybullet, ActiveRobot.B3RobotId, 2);
        bb.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[0], BulletBridge.ClipAngle(roll) * Mathf.Deg2Rad);
        bb.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[1], BulletBridge.ClipAngle(yaw) * Mathf.Deg2Rad);
        bb.SetJointPosition(ref cmd, ActiveRobot.B3RobotId, headJointIds[2], BulletBridge.ClipAngle(pitch) * Mathf.Deg2Rad);

        var status = NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, cmd);
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

        IntPtr wakeUpCommand = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(wakeUpCommand, nextSyncedRobot.B3RobotId,
            (int) DynamicsActivationState.eActivationStateDisableSleeping | (int) DynamicsActivationState.eActivationStateWakeUp);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, wakeUpCommand);
        nextSyncedRobot.IsActive = true;

        // ToDo: Work out best way. 0.03 should give enough room for initial synchronisation. Still a bit laggy.
        yield return new WaitForSeconds(0.03f);

        prevSyncedRobot.IsActive = false;
        IntPtr sleepCommand = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand, prevSyncedRobot.B3RobotId, (int) DynamicsActivationState.eActivationStateEnableSleeping | (int) DynamicsActivationState.eActivationStateSleep);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, sleepCommand);

        yield return new WaitForSeconds(0.25f); // ToDo: May be possible to chose even smaller. Don't know though why it is necessary

        IntPtr sleepCommand2 = NativeMethods.b3InitChangeDynamicsInfo(_pybullet);
        NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand2, prevSyncedRobot.B3RobotId, (int) DynamicsActivationState.eActivationStateSleep);
        NativeMethods.b3SubmitClientCommandAndWaitStatus(_pybullet, sleepCommand2);

        yield return 1;
    }

    #endregion
}
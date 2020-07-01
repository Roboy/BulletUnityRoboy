using System;
using System.Collections.Generic;
using System.Threading;
using Controller.Helper;
using UnityEngine;

namespace Controller
{
    public class BulletSyncController : MonoBehaviour
    {
        #region Inspector

        [SerializeField] private List<SyncedRobotInformation> syncedRobots;

        #endregion

        private BulletBridge _bulletBridge;
        private BulletRobot _bulletRobot;
        private LimitationController _limitationController;

        private Thread _stateSyncThread;

        private float _currentTime = 0.0f;

        void Start()
        {
            _bulletBridge = GameObject.FindGameObjectWithTag("BulletConnectionController").GetComponent<BulletBridge>();
            _bulletRobot = GameObject.FindGameObjectWithTag("BulletRobotController").GetComponent<BulletRobot>();

            _limitationController = GameObject.FindGameObjectWithTag("LimitationController").GetComponent<LimitationController>();

            // Start state synchronisation
            _stateSyncThread = new Thread(StateSyncThread);
            _stateSyncThread.Start();
        }

        void Update()
        {
            _currentTime = Time.realtimeSinceStartup * 1000.0f;
        }

        private void OnApplicationQuit()
        {
            _stateSyncThread.Abort();
        }

        /**
         * This function is executed on a different thread. It handles all communication with Bullet server.
         */
        private void StateSyncThread()
        {
            while (_stateSyncThread.IsAlive)
            {
                // IK
                foreach (SyncedIkTargetInformation ikTarget in _bulletRobot.SyncedIkTargetInformations)
                {
                    var inverseKinematicsCommand = NativeMethods.b3CalculateInverseKinematicsCommandInit(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId);
                    NativeMethods.b3CalculateInverseKinematicsAddTargetPositionWithOrientation(inverseKinematicsCommand, ikTarget.EndeffectorLinkId, ref ikTarget.targetPos[0], ref ikTarget.targetOrn[0]);
                    var statusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, inverseKinematicsCommand);

                    int dofCount = 0;
                    double[] jointTargets = new double[_bulletRobot.FreeJoints.Count];
                    int bodyId = _bulletRobot.ActiveRobot.B3RobotId;

                    NativeMethods.b3GetStatusInverseKinematicsJointPositions(statusHandle, ref bodyId, ref dofCount, ref jointTargets[0]);

                    foreach (SyncedRobotInformation syncedRobotInformation in _bulletRobot.SyncedRobots.FindAll(item => item.IsLoaded && item.IsActive))
                    {
                        inverseKinematicsCommand = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, syncedRobotInformation.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                        for (int i = 0; i < jointTargets.Length; i++)
                        {
                            if (!ikTarget.KinematicChain.Contains(_bulletRobot.FreeJoints[i]))
                            {
                                continue;
                            }

                            _bulletBridge.SetJointPosition(ref inverseKinematicsCommand, syncedRobotInformation.B3RobotId, _bulletRobot.FreeJoints[i], jointTargets[i]);
                        }

                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, inverseKinematicsCommand);
                    }
                }

                // Update State
                IntPtr actualStateCommand = NativeMethods.b3RequestActualStateCommandInit(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId);
                actualStateCommand = NativeMethods.b3RequestActualStateCommandInit2(actualStateCommand, _bulletRobot.ActiveRobot.B3RobotId);

                IntPtr jointStatusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, actualStateCommand);
                EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(jointStatusHandle);
                if (statusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
                {
                    foreach (var jointToSync in _bulletRobot.JointsToSync)
                    {
                        b3JointSensorState b3JointSensorState = new b3JointSensorState();
                        NativeMethods.b3GetJointState(_bulletBridge.Pybullet, jointStatusHandle, jointToSync.JointIndex, ref b3JointSensorState);
                        b3JointSensorStateWrapper b3JointSensorStateWrapper = new b3JointSensorStateWrapper(b3JointSensorState, _currentTime + _limitationController.TrackingDelay);
                        lock (jointToSync)
                        {
                            jointToSync.b3JointSensorStates.Enqueue(b3JointSensorStateWrapper);
                        }
                    }
                }

                // Track Head
                IntPtr cmd = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                _bulletBridge.SetJointPosition(ref cmd, _bulletRobot.ActiveRobot.B3RobotId, _bulletRobot.HeadJointIds[0], BulletBridge.ClipAngle(_bulletRobot.SyncedHeadInformation.Roll) * Mathf.Deg2Rad);
                _bulletBridge.SetJointPosition(ref cmd, _bulletRobot.ActiveRobot.B3RobotId, _bulletRobot.HeadJointIds[1], BulletBridge.ClipAngle(_bulletRobot.SyncedHeadInformation.Yaw) * Mathf.Deg2Rad);
                _bulletBridge.SetJointPosition(ref cmd, _bulletRobot.ActiveRobot.B3RobotId, _bulletRobot.HeadJointIds[2], BulletBridge.ClipAngle(_bulletRobot.SyncedHeadInformation.Pitch) * Mathf.Deg2Rad);
                NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmd);

                // Switch Robot Control
                if (_limitationController.SwitchRobot == true)
                {
                    _limitationController.SwitchRobot = false;
                    _bulletRobot.SyncedRobotSwitchInformation.SwitchRobotFlag = true;
                }

                if (_bulletRobot.SyncedRobotSwitchInformation.SwitchRobotFlag || _bulletRobot.SyncedRobotSwitchInformation.WaitCounterA != 0 || _bulletRobot.SyncedRobotSwitchInformation.WaitCounterB != 0)
                {
                    if (_bulletRobot.SyncedRobotSwitchInformation.SwitchRobotFlag)
                    {
                        _bulletRobot.SyncedRobotSwitchInformation.SwitchRobotFlag = false;
                        _bulletRobot.SyncedRobotSwitchInformation.WaitCounterA = 0;
                        _bulletRobot.SyncedRobotSwitchInformation.WaitCounterB = 0;

                        int currentRobotArrIndex = _bulletRobot.SyncedRobots.IndexOf(_bulletRobot.ActiveRobot);
                        if (currentRobotArrIndex + 1 >= _bulletRobot.SyncedRobots.Count)
                        {
                            continue;
                        }

                        _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot = _bulletRobot.SyncedRobots[currentRobotArrIndex];
                        _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot = _bulletRobot.SyncedRobots[currentRobotArrIndex + 1];

                        Debug.Log("Switching Robot from " + _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.B3RobotId + " to " + _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.B3RobotId);

                        IntPtr wakeUpCommand = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
                        NativeMethods.b3ChangeDynamicsInfoSetActivationState(wakeUpCommand, _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.B3RobotId,
                            (int) DynamicsActivationState.eActivationStateDisableSleeping | (int) DynamicsActivationState.eActivationStateWakeUp);
                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, wakeUpCommand);
                        _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.IsActive = true;
                    }

                    // Happens after 15*[_stateSyncUpdateRate], to give time for the simulation to update state
                    if (_bulletRobot.SyncedRobotSwitchInformation.WaitCounterA == 15)
                    {
                        _bulletRobot.SyncedRobotSwitchInformation.WaitCounterA = -1;

                        // The next robot moves to the position of the old robot
                        _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.Position = _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position;

                        // Old robot moves to an (arbitray) different position, this robot is not used anymore
                        _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position = new Vector3(
                            _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position.x - ((_bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.B3RobotId + 1) * 1.5f),
                            _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position.y, _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position.z);

                        // Move old robot to its new position
                        IntPtr movePrevRobotCmdHandle = NativeMethods.b3CreatePoseCommandInit(_bulletBridge.Pybullet, _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.B3RobotId);
                        NativeMethods.b3CreatePoseCommandSetBasePosition(movePrevRobotCmdHandle, _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position.x,
                            _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position.y,
                            _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.Position.z);
                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, movePrevRobotCmdHandle);

                        // Move new robot to its new position
                        IntPtr moveNextRobotCmdHandle = NativeMethods.b3CreatePoseCommandInit(_bulletBridge.Pybullet, _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.B3RobotId);
                        NativeMethods.b3CreatePoseCommandSetBasePosition(moveNextRobotCmdHandle, _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.Position.x,
                            _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.Position.y,
                            _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot.Position.z + 0.625); // ToDo: Why is the Z value not right?
                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, moveNextRobotCmdHandle);

                        _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.IsActive = false;
                        IntPtr sleepCommand = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
                        NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand, _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.B3RobotId,
                            (int) DynamicsActivationState.eActivationStateEnableSleeping | (int) DynamicsActivationState.eActivationStateSleep);
                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, sleepCommand);
                        
                        // Eventually apply MaxVelocity
                        _limitationController.UpdateVelocity = true;
                    }

                    if (_bulletRobot.SyncedRobotSwitchInformation.WaitCounterA != -1)
                    {
                        _bulletRobot.SyncedRobotSwitchInformation.WaitCounterA++;
                    }

                    // Happens after 30*[_stateSyncUpdateRate]
                    // Not sure, why necessary. But if the state command is not set again after a bit of time, performance decreases.
                    if (_bulletRobot.SyncedRobotSwitchInformation.WaitCounterB == 30)
                    {
                        _bulletRobot.SyncedRobotSwitchInformation.WaitCounterB = -1;

                        IntPtr sleepCommand2 = NativeMethods.b3InitChangeDynamicsInfo(_bulletBridge.Pybullet);
                        NativeMethods.b3ChangeDynamicsInfoSetActivationState(sleepCommand2, _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot.B3RobotId, (int) DynamicsActivationState.eActivationStateSleep);
                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, sleepCommand2);

                        _bulletRobot.SyncedRobotSwitchInformation.PrevSyncedRobot = null;
                        _bulletRobot.SyncedRobotSwitchInformation.NextSyncedRobot = null;
                    }

                    if (_bulletRobot.SyncedRobotSwitchInformation.WaitCounterB != -1)
                    {
                        _bulletRobot.SyncedRobotSwitchInformation.WaitCounterB++;
                    }
                }

                // Change Max Velocity
                if (_limitationController.UpdateVelocity == true)
                {
                    _limitationController.UpdateVelocity = false;

                    Debug.Log("Updating MaxVelocity to " + _limitationController.MaxVelocity);

                    foreach (var freeJoint in _bulletRobot.FreeJoints)
                    {
                        b3JointInfo b3JointInfo = new b3JointInfo();
                        NativeMethods.b3GetJointInfo(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId, freeJoint, ref b3JointInfo);

                        IntPtr cmdHandle = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                        NativeMethods.b3JointControlSetMaximumVelocity(cmdHandle, b3JointInfo.m_uIndex, _limitationController.MaxVelocity);
                        NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, cmdHandle);
                    }
                }
            }
        }
    }
}
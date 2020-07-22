﻿using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using Bullet;
using Bullet.Helper;
using Controller.Helper;
using RosSharp;
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
        private List<BulletObject> _bulletObjects = new List<BulletObject>();

        private Thread _stateSyncThread;

        private float _currentTime = 0.0f;

        void Start()
        {
            _bulletBridge = GameObject.FindGameObjectWithTag("BulletConnectionController").GetComponent<BulletBridge>();
            _bulletRobot = GameObject.FindGameObjectWithTag("BulletRobotController").GetComponent<BulletRobot>();

            _limitationController = GameObject.FindGameObjectWithTag("LimitationController").GetComponent<LimitationController>();

            GameObject[] bulletObjectGameObjects = GameObject.FindGameObjectsWithTag("BulletObject");
            foreach (GameObject bulletObjectGameObject in bulletObjectGameObjects)
            {
                BulletObject bulletObject = bulletObjectGameObject.GetComponent<BulletObject>();
                if (bulletObject)
                {
                    _bulletObjects.Add(bulletObject);
                }
            }

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
                #region Tracking: Inverse Kinematics

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

                #endregion

                #region Tracking: Update Robot Joints

                // Update State
                IntPtr actualRobotStateCommand = NativeMethods.b3RequestActualStateCommandInit(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId);
                actualRobotStateCommand = NativeMethods.b3RequestActualStateCommandInit2(actualRobotStateCommand, _bulletRobot.ActiveRobot.B3RobotId);

                IntPtr jointStatusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, actualRobotStateCommand);
                EnumSharedMemoryServerStatus statusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(jointStatusHandle);
                if (statusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
                {
                    foreach (var jointToSync in _bulletRobot.JointsToSync)
                    {
                        // Update Joint
                        b3JointSensorState b3JointSensorState = new b3JointSensorState();
                        NativeMethods.b3GetJointState(_bulletBridge.Pybullet, jointStatusHandle, jointToSync.JointIndex, ref b3JointSensorState);

                        // Update Link
                        b3LinkState b3LinkState = new b3LinkState();
                        NativeMethods.b3GetLinkState(_bulletBridge.Pybullet, jointStatusHandle, jointToSync.B3LinkIndex, ref b3LinkState);

                        b3JointSensorStateWrapper b3JointSensorStateWrapper = new b3JointSensorStateWrapper(b3JointSensorState, b3LinkState, _currentTime + _limitationController.TrackingDelay);
                        lock (jointToSync)
                        {
                            jointToSync.b3JointSensorStates.Enqueue(b3JointSensorStateWrapper);
                        }
                    }
                }

                #endregion

                #region Tracking: Head Movements

                // Track Head
                IntPtr trackHeadCommand = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                _bulletBridge.SetJointPosition(ref trackHeadCommand, _bulletRobot.ActiveRobot.B3RobotId, _bulletRobot.HeadJointIds[0], BulletBridge.ClipAngle(_bulletRobot.SyncedHeadInformation.Roll) * Mathf.Deg2Rad);
                _bulletBridge.SetJointPosition(ref trackHeadCommand, _bulletRobot.ActiveRobot.B3RobotId, _bulletRobot.HeadJointIds[1], BulletBridge.ClipAngle(_bulletRobot.SyncedHeadInformation.Yaw) * Mathf.Deg2Rad);
                _bulletBridge.SetJointPosition(ref trackHeadCommand, _bulletRobot.ActiveRobot.B3RobotId, _bulletRobot.HeadJointIds[2], BulletBridge.ClipAngle(_bulletRobot.SyncedHeadInformation.Pitch) * Mathf.Deg2Rad);
                NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, trackHeadCommand);

                #endregion

                #region Tracking: Fingers

                // Track Fingers
                IntPtr trackFingersCommand = NativeMethods.b3JointControlCommandInit2(_bulletBridge.Pybullet, _bulletRobot.ActiveRobot.B3RobotId, (int) EnumControlMode.CONTROL_MODE_POSITION_VELOCITY_PD);
                foreach (SyncedGloveInformation bulletRobotSyncedGloveInformation in _bulletRobot.SyncedGloveInformations)
                {
                    foreach (List<SyncedGloveJoint> syncedGloveJoints in bulletRobotSyncedGloveInformation.HandJointInformation)
                    {
                        foreach (SyncedGloveJoint syncedGloveJoint in syncedGloveJoints)
                        {
                            _bulletBridge.SetJointPosition(ref trackFingersCommand, _bulletRobot.ActiveRobot.B3RobotId, syncedGloveJoint.JointIndex, syncedGloveJoint.TargetJointPos);
                        }
                    }
                }

                NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, trackFingersCommand);

                #endregion

                #region SenseGlove: Finger Collisions

                b3ContactInformation b3ContactInformation = new b3ContactInformation();
                IntPtr collisionCommand = NativeMethods.b3InitRequestContactPointInformation(_bulletBridge.Pybullet);
                NativeMethods.b3SetContactFilterBodyA(collisionCommand, _bulletRobot.ActiveRobot.B3RobotId);
                // NativeMethods.b3SetContactFilterLinkA(); // ToDo: Eventually only for specific finger links
                NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, collisionCommand);

                NativeMethods.b3GetContactPointInformation(_bulletBridge.Pybullet, ref b3ContactInformation);

                for (int i = 0; i < b3ContactInformation.m_numContactPoints; i++)
                {
                    b3ContactPointData b3ContactPointData = (b3ContactPointData) Marshal.PtrToStructure(b3ContactInformation.m_contactPointData, typeof(b3ContactPointData));
                    //Debug.Log(JsonUtility.ToJson(b3ContactPointData));
                }

                #endregion

                #region Limitation: Switch Robot Control

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

                #endregion

                #region Limitation: Maximum Velocity

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

                #endregion

                #region Bullet Objects: Instantiation

                foreach (BulletObject bulletObject in _bulletObjects)
                {
                    if (!bulletObject.BulletBodyInformation.Instantiated)
                    {
                        bulletObject.BulletBodyInformation.Instantiated = true;
                        bulletObject.BulletBodyInformation.BodyId = _bulletBridge.LoadURDF(bulletObject.BulletBodyInformation.UrdfPadth, bulletObject.BulletBodyInformation.Position.Unity2Ros(),
                            bulletObject.BulletBodyInformation.Rotation.Unity2Ros(), bulletObject.BulletBodyInformation.Scaling, bulletObject.BulletBodyInformation.IsStatic ? 1 : 0);
                        Debug.Log("[Object] Instantiated " + bulletObject.BulletBodyInformation.UrdfPadth + " with bodyId " + bulletObject.BulletBodyInformation.BodyId);
                    }
                }

                #endregion

                #region Bullet Objects: Position / Rotation

                foreach (BulletObject bulletObject in _bulletObjects)
                {
                    if (!bulletObject.BulletBodyInformation.IsStatic && bulletObject.BulletBodyInformation.Instantiated)
                    {
                        IntPtr actualObjectStateCommand = NativeMethods.b3RequestActualStateCommandInit(_bulletBridge.Pybullet, bulletObject.BulletBodyInformation.BodyId);
                        IntPtr actualObjectStateStatusHandle = NativeMethods.b3SubmitClientCommandAndWaitStatus(_bulletBridge.Pybullet, actualObjectStateCommand);

                        EnumSharedMemoryServerStatus actualObjectStateStatusType = (EnumSharedMemoryServerStatus) NativeMethods.b3GetStatusType(actualObjectStateStatusHandle);
                        if (actualObjectStateStatusType == EnumSharedMemoryServerStatus.CMD_ACTUAL_STATE_UPDATE_COMPLETED)
                        {
                            int bodyUniqueId = -1;
                            int numDegreeOfFreedomQ = 0;
                            int numDegreeOfFreedomU = 0;
                            IntPtr rootLocalInertialFrame = IntPtr.Zero;
                            IntPtr actualStateQ = IntPtr.Zero;
                            IntPtr actualStateQdot = IntPtr.Zero;
                            IntPtr jointReactionForces = IntPtr.Zero;

                            NativeMethods.b3GetStatusActualState(actualObjectStateStatusHandle, ref bodyUniqueId, ref numDegreeOfFreedomQ, ref numDegreeOfFreedomU, ref rootLocalInertialFrame, ref actualStateQ,
                                ref actualStateQdot, ref jointReactionForces);

                            BulletPosition bulletPosition = (BulletPosition) Marshal.PtrToStructure(actualStateQ, typeof(BulletPosition));

                            Vector3 newPos = new Vector3((float) bulletPosition.x, (float) bulletPosition.y, (float) bulletPosition.z).Ros2Unity();
                            Quaternion newOrn = new Quaternion((float) bulletPosition.qx, (float) bulletPosition.qy, (float) bulletPosition.qz, (float) bulletPosition.qw).Ros2Unity();

                            bulletObject.BulletBodyInformation.Position = newPos;
                            bulletObject.BulletBodyInformation.Rotation = newOrn;
                        }
                    }
                }

                #endregion
            }
        }
    }
}
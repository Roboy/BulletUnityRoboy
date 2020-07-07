using System;
using System.Collections.Generic;
using JetBrains.Annotations;
using RosSharp.Urdf;

namespace Controller.Helper
{
    /**
     * Class holds information about all joints that have to be synced from Bullet to Unity.
     */
    public class SyncedJointsInformation
    {
        private readonly int _b3JointIndex;
        private readonly int _b3LinkIndex;
        private readonly UrdfJoint _urdfJoint;
        private readonly int _jointIndex;
        private readonly String _jointName;
        private readonly UrdfLink _urdfLink;

        public Queue<b3JointSensorStateWrapper> b3JointSensorStates = new Queue<b3JointSensorStateWrapper>();

        public UrdfJoint UrdfJoint => _urdfJoint;

        public int JointIndex => _jointIndex;

        public string JointName => _jointName;

        public int B3JointIndex => _b3JointIndex;

        public UrdfLink UrdfLink => _urdfLink;

        public int B3LinkIndex => _b3LinkIndex;

        public SyncedJointsInformation(int b3JointIndex, [NotNull] UrdfJoint urdfJoint, int jointIndex, string jointName, int b3LinkIndex, UrdfLink urdfLink)
        {
            this._urdfJoint = urdfJoint ? urdfJoint : throw new ArgumentNullException(nameof(urdfJoint));
            this._jointIndex = jointIndex;
            this._jointName = jointName;
            this._b3JointIndex = b3JointIndex;
            this._urdfLink = urdfLink;
            this._b3LinkIndex = b3LinkIndex;
        }
    }
}
using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace Controller.Helper
{
    /**
     * Class to store different robot urdf paths. As of now, angle limitations cannot be modified during runtime. Therefore spawn multiple robots in simulation and sync with one.
     */
    [Serializable]
    public class SyncedRobotInformation
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
        private float _scaling = .75f;
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

        public float Scaling => _scaling;

        public Vector3 TrackingPosition
        {
            get => _trackingPosition;
            set => _trackingPosition = value;
        }
    }
}
using System.Collections.Generic;
using Controller.Helper;
using SenseGloveCs.Kinematics;
using UnityEngine;

namespace Bullet.Helper
{
    public class BulletBodyInformation
    {
        private int _bodyId;
        private string _urdfPath;
        private bool _instantiated = true;
        private bool _isStatic = true;
        private bool _isGraspable = false;

        private Vector3 _position;
        private Quaternion _rotation;
        private double _scaling;

        private Vector3 _newPosition;
        private Quaternion _newRotation;
        private bool _updatePositionAndRotation = false;
        
        public Queue<b3ObjectStateWrapper> b3ObjectStates = new Queue<b3ObjectStateWrapper>();

        public int BodyId
        {
            get => _bodyId;
            set => _bodyId = value;
        }

        public string UrdfPadth
        {
            get => _urdfPath;
            set => _urdfPath = value;
        }

        public bool Instantiated
        {
            get => _instantiated;
            set => _instantiated = value;
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

        public double Scaling
        {
            get => _scaling;
            set => _scaling = value;
        }

        public bool IsStatic
        {
            get => _isStatic;
            set => _isStatic = value;
        }

        public bool IsGraspable
        {
            get => _isGraspable;
            set => _isGraspable = value;
        }

        public Vector3 NewPosition
        {
            get => _newPosition;
            set => _newPosition = value;
        }

        public Quaternion NewRotation
        {
            get => _newRotation;
            set => _newRotation = value;
        }

        public bool UpdatePositionAndRotation
        {
            get => _updatePositionAndRotation;
            set => _updatePositionAndRotation = value;
        }
    }
}
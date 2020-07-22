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

        private Vector3 position;
        private Quaternion rotation;
        private double scaling;

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
            get => position;
            set => position = value;
        }

        public Quaternion Rotation
        {
            get => rotation;
            set => rotation = value;
        }

        public double Scaling
        {
            get => scaling;
            set => scaling = value;
        }

        public bool IsStatic
        {
            get => _isStatic;
            set => _isStatic = value;
        }
    }
}
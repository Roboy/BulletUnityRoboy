using UnityEngine;

namespace Controller.Helper
{
    /**
     * Class holds information about head movement.
     */
    public class SyncedHeadInformation
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
}
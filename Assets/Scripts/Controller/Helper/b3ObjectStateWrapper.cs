using UnityEngine;

namespace Controller.Helper
{
    public class b3ObjectStateWrapper
    {
        public readonly Vector3 objectPosition;
        public readonly Quaternion objectRotation;
        public readonly float atTime;

        public b3ObjectStateWrapper(Vector3 objectPosition, Quaternion objectRotation, float atTime)
        {
            this.objectPosition = objectPosition;
            this.objectRotation = objectRotation;
            this.atTime = atTime;
        }
    }
}
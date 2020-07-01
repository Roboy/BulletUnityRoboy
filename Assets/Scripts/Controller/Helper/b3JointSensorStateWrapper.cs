namespace Controller.Helper
{
    public class b3JointSensorStateWrapper
    {
        public readonly b3JointSensorState b3JointSensorState;
        public readonly float atTime;

        public b3JointSensorStateWrapper(b3JointSensorState b3JointSensorState, float atTime)
        {
            this.b3JointSensorState = b3JointSensorState;
            this.atTime = atTime;
        }
    }
}
namespace Controller.Helper
{
    public class b3JointSensorStateWrapper
    {
        public readonly b3JointSensorState b3JointSensorState;
        public readonly b3LinkState b3LinkState;
        public readonly float atTime;

        public b3JointSensorStateWrapper(b3JointSensorState b3JointSensorState, b3LinkState b3LinkState, float atTime)
        {
            this.b3JointSensorState = b3JointSensorState;
            this.b3LinkState = b3LinkState;
            this.atTime = atTime;
        }
    }
}
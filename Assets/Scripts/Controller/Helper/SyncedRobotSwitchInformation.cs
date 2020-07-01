namespace Controller.Helper
{
    public class SyncedRobotSwitchInformation
    {
        private bool _switchRobotFlag;
        private int _waitCounterA;
        private int _waitCounterB;
        private SyncedRobotInformation _prevSyncedRobot;
        private SyncedRobotInformation _nextSyncedRobot;

        public SyncedRobotSwitchInformation()
        {
            _switchRobotFlag = false;
            _waitCounterA = -1;
            _waitCounterB = -1;
            _prevSyncedRobot = null;
            _nextSyncedRobot = null;
        }

        public bool SwitchRobotFlag
        {
            get => _switchRobotFlag;
            set => _switchRobotFlag = value;
        }

        public int WaitCounterA
        {
            get => _waitCounterA;
            set => _waitCounterA = value;
        }

        public int WaitCounterB
        {
            get => _waitCounterB;
            set => _waitCounterB = value;
        }

        public SyncedRobotInformation PrevSyncedRobot
        {
            get => _prevSyncedRobot;
            set => _prevSyncedRobot = value;
        }

        public SyncedRobotInformation NextSyncedRobot
        {
            get => _nextSyncedRobot;
            set => _nextSyncedRobot = value;
        }
    }
}
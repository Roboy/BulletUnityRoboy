using System.Collections.Generic;
using UnityEngine;

namespace Controller.Helper
{
    /**
     * Class holds information for joints that are tracked (used for IK calculation)
     */
    public class SyncedIkTargetInformation
    {
        public SyncedIkTargetInformation(Transform transform, int endeffectorLinkId, List<int> kinematicChain)
        {
            this._transform = transform;
            this._endeffectorLinkId = endeffectorLinkId;
            this._kinematicChain = kinematicChain;
        }

        private readonly Transform _transform;
        private readonly int _endeffectorLinkId;
        private readonly List<int> _kinematicChain;

        public double[] targetPos = {0, 0, 0};
        public double[] targetOrn = {0, 0, 0, 0};

        public Transform Transform => _transform;

        public int EndeffectorLinkId => _endeffectorLinkId;

        public List<int> KinematicChain => _kinematicChain;
    }
}
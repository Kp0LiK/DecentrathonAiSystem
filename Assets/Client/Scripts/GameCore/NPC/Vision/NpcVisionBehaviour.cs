using CustomTools.Updater;
using UnityEngine;

namespace Client
{
    public class NpcVisionBehaviour : MonoBehaviour
    {
        [SerializeField] private CarAiBehaviour _carAiBehaviour;
        [SerializeField] private UpdaterMono _updaterMono;
        [SerializeField] private float _visionRange;

        [SerializeField] private VisionItem[] _visions;

        private NpcVisionUpdater _visionUpdater;
        private readonly LayerMask _obstacleMask = 1 << 6;


        private void Awake()
        {
            _visionUpdater = new NpcVisionUpdater(_carAiBehaviour, transform, _visionRange, "IgnoreVision", _visions);
            _updaterMono.Add(_visionUpdater);
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.green;
            foreach (var vision in _visions)
            {
                if (vision.Weight == 0)
                {
                    Gizmos.DrawRay(vision.Point.position, vision.Point.forward * _visionRange);
                }
            }
        }
    }
}
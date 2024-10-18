using CustomTools.Updater;
using UnityEngine;

namespace Client
{
    public class NpcVisionBehaviour : MonoBehaviour
    {
        [SerializeField] private UpdaterMono _updaterMono;
        [SerializeField] private float _visionRange;
        [SerializeField] private float _offsetX;
        [SerializeField] private float _offsetY;
        [SerializeField] private float _spaceBetweenLines;

        private NpcVisionUpdater _visionUpdater;
        private readonly LayerMask _obstacleMask = 1 << 6;

        private void Awake()
        {
            _visionUpdater = new NpcVisionUpdater(transform, _visionRange, _offsetX, _offsetY, _spaceBetweenLines,
                _obstacleMask);
            _updaterMono.Add(_visionUpdater);
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;

            var npcPosition = transform.position;

            for (var i = 0; i < 4; i++)
            {
                var offset = (i - 1.5f) * _spaceBetweenLines;
                var startPosition = npcPosition + new Vector3(_offsetX * offset, _offsetY * offset, 0);
                var endPosition = startPosition + transform.forward * _visionRange;
                Gizmos.DrawLine(startPosition, endPosition);
            }
        }
    }
}
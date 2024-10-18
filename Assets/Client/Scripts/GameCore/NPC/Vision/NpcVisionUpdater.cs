using CustomTools.Updater;
using UnityEngine;

namespace Client
{
    public class NpcVisionUpdater : IUpdateMono
    {
        private readonly Transform _transform;
        private readonly float _visionRange;
        private readonly float _offsetX;
        private readonly float _offsetY;
        private readonly float _spaceBetweenLines;
        private readonly LayerMask _obstacleLayerMask;

        public NpcVisionUpdater(Transform transform, float visionRange, float offsetX, float offsetY,
            float spaceBetweenLines, LayerMask obstacleLayerMask)
        {
            _transform = transform;
            _visionRange = visionRange;
            _offsetX = offsetX;
            _offsetY = offsetY;
            _spaceBetweenLines = spaceBetweenLines;
            _obstacleLayerMask = obstacleLayerMask;
        }


        public void Tick()
        {
            PerformRaycastCheck();
        }

        private void PerformRaycastCheck()
        {
            var npcPosition = _transform.position;

            for (var i = 0; i < 4; i++)
            {
                var offset = (i - 1.5f) * _spaceBetweenLines;

                var startPosition = npcPosition + new Vector3(_offsetX * offset, _offsetY * offset, 0);
                var direction = _transform.forward;

                if (Physics.Raycast(startPosition, direction, out var hit, _visionRange, _obstacleLayerMask))
                {
                    Debug.Log($"Line {i} detected obstacle: {hit.collider.gameObject.name} at distance {hit.distance}");
                }
                else
                {
                    Debug.Log($"Line {i} detected no obstacles");
                }

                Debug.DrawRay(startPosition, direction * _visionRange, Color.red);
            }
        }
    }
}
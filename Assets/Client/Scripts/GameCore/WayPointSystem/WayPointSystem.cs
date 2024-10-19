using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace Client
{
    public class WayPointSystem : MonoBehaviour
    {
        [SerializeField] private List<Vector3> _waypoints = new();
        [SerializeField] private float _distanceBetweenPoints = 5f;
        [SerializeField] private float _maxPathLength;

        public List<Vector3> Waypoints => _waypoints;

        private void Start()
        {
            GenerateWaypoints(transform.position);
        }

        public void GenerateWaypoints(Vector3 startPosition)
        {
            _waypoints.Clear();

            Vector3 currentPosition = startPosition;
            Vector3 direction = transform.forward;

            float distanceTraveled = 0f;

            _waypoints.Add(currentPosition);

            while (distanceTraveled < _maxPathLength)
            {
                Vector3 nextPosition = currentPosition + direction * _distanceBetweenPoints;

                NavMeshHit hit;
                if (NavMesh.SamplePosition(nextPosition, out hit, _distanceBetweenPoints * 2f, NavMesh.AllAreas))
                {
                    _waypoints.Add(hit.position);

                    currentPosition = hit.position;
                    direction = (hit.position - currentPosition).normalized;

                    distanceTraveled += _distanceBetweenPoints;

                    if (Vector3.Distance(hit.position, startPosition) < _distanceBetweenPoints && _waypoints.Count > 10)
                    {
                        Debug.Log("Замкнули круг.");
                        break;
                    }
                }
                else
                {
                    direction = Quaternion.Euler(0, Random.Range(-15f, 15f), 0) * direction;
                    Debug.LogWarning("Корректировка направления.");
                }
            }

            Debug.Log("Точек сгенерировано: " + _waypoints.Count);
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.yellow;
            foreach (var point in _waypoints)
            {
                Gizmos.DrawSphere(point, 0.5f);
            }

            if (_waypoints.Count > 1)
            {
                for (int i = 0; i < _waypoints.Count - 1; i++)
                {
                    Gizmos.DrawLine(_waypoints[i], _waypoints[i + 1]);
                }

                Gizmos.DrawLine(_waypoints[^1], _waypoints[0]);
            }
        }
    }
}
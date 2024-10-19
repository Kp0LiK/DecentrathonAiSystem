using UnityEngine;
using System.Collections.Generic;

namespace Client
{
    public class NavMeshWaypointSystem : MonoBehaviour
    {
        [SerializeField] private float _waypointSpacing = 10f;
        [SerializeField] private List<Vector3> _waypoints = new();

        [SerializeField] private List<Transform> _waypointObjects = new();

        public List<Vector3> Waypoints => _waypoints;

        private void Start()
        {
            AddWaypointsFromTransforms();
        }

        private void AddWaypointFromTransform(Transform waypointTransform)
        {
            var newWaypoint = waypointTransform.position;

            if (!IsPointCloseToWaypoints(newWaypoint))
            {
                _waypoints.Add(newWaypoint);
                Debug.Log($"Точка добавлена: {newWaypoint}");
            }
            else
            {
                Debug.LogWarning("Точка слишком близка к существующим путевым точкам.");
            }
        }

        private void AddWaypointsFromTransforms()
        {
            foreach (Transform waypoint in _waypointObjects)
            {
                AddWaypointFromTransform(waypoint);
            }
        }

        public void RemoveWaypoint(Transform waypointTransform)
        {
            Vector3 point = waypointTransform.position;

            if (_waypoints.Contains(point))
            {
                _waypoints.Remove(point);
                Debug.Log($"Точка удалена: {point}");
            }
            else
            {
                Debug.LogWarning("Точка не найдена в списке.");
            }
        }

        private bool IsPointCloseToWaypoints(Vector3 point)
        {
            foreach (var waypoint in _waypoints)
            {
                if (Vector3.Distance(point, waypoint) < _waypointSpacing)
                {
                    return true;
                }
            }
            return false;
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            foreach (var waypoint in _waypoints)
            {
                Gizmos.DrawSphere(waypoint, 0.5f);
            }
        }
    }
}

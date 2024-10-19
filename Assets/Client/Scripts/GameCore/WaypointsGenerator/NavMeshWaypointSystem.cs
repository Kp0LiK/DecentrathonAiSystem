using Unity.AI.Navigation;
using UnityEngine;
using UnityEngine.AI;
using System.Collections.Generic;

namespace Client
{
    public class NavMeshWaypointSystem : MonoBehaviour
    {
        [SerializeField] private float _waypointSpacing = 10f;
        [SerializeField] private NavMeshSurface _navMeshSurface;
        private List<Vector3> _waypoints = new();

        private void Start()
        {
            GenerateWaypointsFromNavMesh();
        }

        void GenerateWaypointsFromNavMesh()
        {
            NavMeshTriangulation navMeshData = NavMesh.CalculateTriangulation();
            HashSet<Vector3> visitedPoints = new HashSet<Vector3>();

            for (int i = 0; i < navMeshData.vertices.Length; i++)
            {
                var point = navMeshData.vertices[i];

                if (!IsPointCloseToWaypoints(point))
                {
                    _waypoints.Add(point);
                    visitedPoints.Add(point);
                    CreatePathToNextWaypoint(point, visitedPoints);
                }
            }
        }

        private void CreatePathToNextWaypoint(Vector3 currentPoint, HashSet<Vector3> visitedPoints)
        {
            if (NavMesh.SamplePosition(currentPoint + transform.forward * _waypointSpacing, out var hit, _waypointSpacing,
                    NavMesh.AllAreas))
            {
                var nextPoint = hit.position;

                if (!visitedPoints.Contains(nextPoint) && !IsPointCloseToWaypoints(nextPoint))
                {
                    _waypoints.Add(nextPoint);
                    visitedPoints.Add(nextPoint);
                    CreatePathToNextWaypoint(nextPoint, visitedPoints);
                }
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
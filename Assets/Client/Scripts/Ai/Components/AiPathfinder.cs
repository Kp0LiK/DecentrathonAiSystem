using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

namespace Client
{
    public class AiPathfinder : IPathfinder
    {
        public List<Vector3> CalculatePath(Vector3 startPosition, Vector3 destination)
        {
            var navMeshPath = new NavMeshPath();
            NavMesh.CalculatePath(startPosition, destination, NavMesh.AllAreas, navMeshPath);
            return new List<Vector3>(navMeshPath.corners);
        }
    }
}
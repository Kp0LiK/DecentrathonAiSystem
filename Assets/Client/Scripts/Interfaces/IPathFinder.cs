using System.Collections.Generic;
using UnityEngine;

namespace Client
{
    public interface IPathfinder
    {
        List<Vector3> CalculatePath(Vector3 startPosition, Vector3 destination);
    }
}
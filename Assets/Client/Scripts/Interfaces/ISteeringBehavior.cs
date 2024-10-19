using UnityEngine;

namespace Client
{
    public interface ISteeringBehavior
    {
        float CalculateSteeringAngle(Transform carTransform, Vector3 targetPosition, float maxSteerAngle);
    }
}
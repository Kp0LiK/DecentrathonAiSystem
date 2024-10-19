using UnityEngine;

namespace Client
{
    public class SimpleSteeringBehaviour : ISteeringBehavior
    {
        public float CalculateSteeringAngle(Transform carTransform, Vector3 targetPosition, float maxSteerAngle)
        {
            var localTarget = carTransform.InverseTransformPoint(targetPosition);
            var angle = Mathf.Atan2(localTarget.x, localTarget.z) * Mathf.Rad2Deg;
            return Mathf.Clamp(angle, -maxSteerAngle, maxSteerAngle);
        }
    }
}
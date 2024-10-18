using UnityEngine;

namespace Client
{
    public class WheelMotorController : IMotorController
    {
        private readonly WheelCollider _wheelRearLeft;
        private readonly WheelCollider _wheelRearRight;

        public WheelMotorController(WheelCollider wheelRearLeft, WheelCollider wheelRearRight)
        {
            _wheelRearLeft = wheelRearLeft;
            _wheelRearRight = wheelRearRight;
        }

        public void ApplyMotorTorque(float torque)
        {
            _wheelRearLeft.motorTorque = torque;
            _wheelRearRight.motorTorque = torque;
        }

        public void ApplyBrakeTorque(float torque)
        {
            _wheelRearLeft.brakeTorque = torque;
            _wheelRearRight.brakeTorque = torque;
        }

        public void ReleaseBrakes()
        {
            _wheelRearLeft.brakeTorque = 0f;
            _wheelRearRight.brakeTorque = 0f;
        }
    }
}
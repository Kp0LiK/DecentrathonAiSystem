namespace Client
{
    public interface IMotorController
    {
        void ApplyMotorTorque(float torque);
        void ApplyBrakeTorque(float torque);
        void ReleaseBrakes();
    }
}
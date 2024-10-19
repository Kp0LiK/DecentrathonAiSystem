using UnityEngine;
using System.Collections.Generic;
using UnityEngine.Serialization;

namespace Client
{
    [RequireComponent(typeof(Rigidbody))]
    public class CarAiBehaviour : MonoBehaviour
    {
        [Header("Wheel Colliders")]
        [SerializeField] private WheelCollider _wheelFrontLeft;
        [SerializeField] private WheelCollider _wheelFrontRight;
        [SerializeField] private WheelCollider _wheelRearLeft;
        [SerializeField] private WheelCollider _wheelRearRight;

        [Header("Car Parameters")]
        [SerializeField] private float _maxMotorTorque = 150f;
        [SerializeField] private float _maxSteerAngle = 30f;
        [SerializeField] private float _maxSpeed = 100f;
        
        [Header("Braking Parameters")]
        [SerializeField] private float _brakingDistance = 10f;
        [SerializeField] private float _maxBrakeTorque = 300f;

        [Header("Point")]
        [SerializeField] private Transform _pointB;

        private IPathfinder _pathfinder;
        private ISteeringBehavior _steeringBehavior;
        private IMotorController _motorController;
        private Rigidbody _rigidbody;

        private List<Vector3> _pathCorners;
        private int _currentCornerIndex = 0;
        
        private const float Force = 3.6f;

        private void Awake()
        {
            _pathfinder = new AiPathfinder();
            _steeringBehavior = new SimpleSteeringBehaviour();
            _motorController = new WheelMotorController(_wheelRearLeft, _wheelRearRight);
            
            _rigidbody = GetComponent<Rigidbody>();
        }

        private void Start()
        {
            if (_pointB == null)
            {
                Debug.LogError("Destination is not assigned.");
                enabled = false;
                return;
            }

            CalculatePath();
        }

        private void FixedUpdate()
        {
            if (_pathCorners == null || _pathCorners.Count == 0)
                return;

            if (_currentCornerIndex >= _pathCorners.Count)
            {
                //_motorController.StopMotor();
                _motorController.ApplyBrakeTorque(_maxBrakeTorque);
                return;
            }

            Vector3 targetPosition = _pathCorners[_currentCornerIndex];
            Vector3 directionToTarget = targetPosition - transform.position;

            if (directionToTarget.magnitude < 2f)
            {
                _currentCornerIndex++;
                return;
            }

            float steerAngle = _steeringBehavior.CalculateSteeringAngle(transform, targetPosition, _maxSteerAngle);
            ApplySteering(steerAngle);
            ControlSpeed(directionToTarget.magnitude);
        }

        private void CalculatePath()
        {
            _pathCorners = _pathfinder.CalculatePath(transform.position, _pointB.position);
            if (_pathCorners == null || _pathCorners.Count == 0)
            {
                Debug.LogError("Failed to calculate path.");
                enabled = false;
            }
        }

        private void ApplySteering(float steerAngle)
        {
            _wheelFrontLeft.steerAngle = steerAngle;
            _wheelFrontRight.steerAngle = steerAngle;
        }

        private void ControlSpeed(float distanceToTarget)
        {
            var currentSpeed = _rigidbody.velocity.magnitude * Force;

            if (distanceToTarget <= _brakingDistance)
            {
                var brakeTorque = Mathf.Lerp(0, _maxBrakeTorque, (_brakingDistance - distanceToTarget) / _brakingDistance);
                _motorController.ApplyBrakeTorque(brakeTorque);
                _motorController.ApplyMotorTorque(0f);
            }
            else
            {
                _motorController.ReleaseBrakes();

                if (currentSpeed < _maxSpeed)
                {
                    _motorController.ApplyMotorTorque(_maxMotorTorque);
                }
                else
                {
                    _motorController.ApplyMotorTorque(0f);
                }
            }
        }

        private void OnDrawGizmos()
        {
            if (_pathCorners != null && _pathCorners.Count > 0)
            {
                Gizmos.color = Color.blue;
                for (var i = 0; i < _pathCorners.Count - 1; i++)
                {
                    Gizmos.DrawLine(_pathCorners[i], _pathCorners[i + 1]);
                }
            }
        }
    }
}
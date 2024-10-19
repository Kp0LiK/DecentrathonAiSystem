using UnityEngine;
using System.Collections.Generic;

namespace Client
{
    [RequireComponent(typeof(Rigidbody))]
    public class CarAiBehaviour : MonoBehaviour
    {
        [SerializeField] private NavMeshWaypointSystem _wayPointSystem;
       
        [Header("Wheel Colliders")] [SerializeField]
        private WheelCollider _wheelFrontLeft;

        [SerializeField] private WheelCollider _wheelFrontRight;
        [SerializeField] private WheelCollider _wheelRearLeft;
        [SerializeField] private WheelCollider _wheelRearRight;

        [Header("Car Parameters")] [SerializeField]
        private float _maxMotorTorque = 150f;

        [SerializeField] private float _maxSteerAngle = 30f;
        [SerializeField] private float _maxSpeed = 100f;
        [SerializeField] private float _minSpeedInTurn = 30f;

        [Header("Braking Parameters")] [SerializeField]
        private float _brakingDistance = 10f;

        [SerializeField] private float _maxBrakeTorque = 300f;

        [Header("Physics Parameters")] [SerializeField]
        private float _tractionControl = 0.8f;

        [SerializeField] private float _downforce = 100f;
        [SerializeField] private float _turnSmoothing = 0.05f;
        [SerializeField] private float _antiRollBarStiffness = 5000f;
        
        private IMotorController _motorController;
        private ISteeringBehavior _steeringBehavior;

        private List<Vector3> _waypoints;
        private int _currentWaypointIndex = 0;
        private Rigidbody _rigidbody;
        private float _currentSpeed = 0f;

        private void Awake()
        {
            _rigidbody = GetComponent<Rigidbody>();

            _motorController = new WheelMotorController(_wheelRearLeft, _wheelRearRight);
            _steeringBehavior = new SimpleSteeringBehaviour();
        }

        private void Start()
        {
            if (_wayPointSystem == null) return;

            _waypoints = _wayPointSystem.Waypoints;
            if (_waypoints.Count > 0)
            {
                SetNextWaypoint();
            }
            else
            {
                Debug.LogError("Не удалось получить точки пути.");
                enabled = false;
            }
        }

        private void FixedUpdate()
        {
            if (_waypoints == null || _waypoints.Count == 0) return;

            var targetPosition = _waypoints[_currentWaypointIndex];
            var distanceToTarget = Vector3.Distance(transform.position, targetPosition);

            var steerAngle = _steeringBehavior.CalculateSteeringAngle(transform, targetPosition, _maxSteerAngle);
            ApplySteering(steerAngle);

            ControlSpeed(distanceToTarget, steerAngle);

            ApplyDownforce();

            TractionControl();

            ApplyAntiRollBar();

            if (distanceToTarget <= _brakingDistance)
            {
                SetNextWaypoint();
            }
        }

        private void ApplySteering(float steerAngle)
        {
            _wheelFrontLeft.steerAngle = steerAngle;
            _wheelFrontRight.steerAngle = steerAngle;
        }

        private void ControlSpeed(float distanceToTarget, float steerAngle)
        {
            _currentSpeed = _rigidbody.velocity.magnitude;

            var speedFactor = Mathf.Clamp01(1f - (Mathf.Abs(steerAngle) / _maxSteerAngle));
            var targetSpeed = Mathf.Lerp(_minSpeedInTurn, _maxSpeed, speedFactor);

            if (distanceToTarget <= _brakingDistance)
            {
                _motorController.ApplyBrakeTorque(_maxBrakeTorque);
            }
            else
            {
                _motorController.ReleaseBrakes();

                if (_currentSpeed < targetSpeed)
                {
                    _motorController.ApplyMotorTorque(_maxMotorTorque);
                }
                else
                {
                    _motorController.ApplyMotorTorque(0f);
                }
            }
        }

        private void ApplyDownforce()
        {
            _rigidbody.AddForce(-transform.up * _downforce * _rigidbody.velocity.magnitude);
        }

        private void TractionControl()
        {
            WheelHit wheelHit;
            _wheelRearLeft.GetGroundHit(out wheelHit);
            if (wheelHit.forwardSlip >= _tractionControl)
            {
                _motorController.ApplyMotorTorque(_maxMotorTorque * (1 - _tractionControl));
            }

            _wheelRearRight.GetGroundHit(out wheelHit);
            if (wheelHit.forwardSlip >= _tractionControl)
            {
                _motorController.ApplyMotorTorque(_maxMotorTorque * (1 - _tractionControl));
            }
        }

        private void ApplyAntiRollBar()
        {
            ApplyAntiRollBarToAxle(_wheelFrontLeft, _wheelFrontRight);
            ApplyAntiRollBarToAxle(_wheelRearLeft, _wheelRearRight);
        }

        private void ApplyAntiRollBarToAxle(WheelCollider wheelL, WheelCollider wheelR)
        {
            var travelL = 1.0f;
            var travelR = 1.0f;

            var groundedL = wheelL.GetGroundHit(out var hit);
            if (groundedL)
                travelL = (-wheelL.transform.InverseTransformPoint(hit.point).y - wheelL.radius) /
                          wheelL.suspensionDistance;

            var groundedR = wheelR.GetGroundHit(out hit);
            if (groundedR)
                travelR = (-wheelR.transform.InverseTransformPoint(hit.point).y - wheelR.radius) /
                          wheelR.suspensionDistance;

            var antiRollForce = (travelL - travelR) * _antiRollBarStiffness;

            if (groundedL)
                _rigidbody.AddForceAtPosition(wheelL.transform.up * -antiRollForce, wheelL.transform.position);
            if (groundedR)
                _rigidbody.AddForceAtPosition(wheelR.transform.up * antiRollForce, wheelR.transform.position);
        }

        private void SetNextWaypoint()
        {
            _currentWaypointIndex = (_currentWaypointIndex + 1) % _waypoints.Count;
        }

        private void OnDrawGizmos()
        {
            if (_waypoints is not { Count: > 0 }) return;
            Gizmos.color = Color.green;
            for (var i = 0; i < _waypoints.Count - 1; i++)
            {
                Gizmos.DrawLine(_waypoints[i], _waypoints[i + 1]);
            }

            Gizmos.DrawLine(_waypoints[^1], _waypoints[0]);
        }
    }
}
using UnityEngine;
using System.Collections.Generic;
using UnityEngine.AI;

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

        [Header("Waypoint System")]
        [SerializeField] private WayPointSystem _wayPointSystem; // Ссылка на WayPointSystem для получения точек

        private List<Vector3> _waypoints; // Список путевых точек
        private int _currentWaypointIndex = 0;

        private IMotorController _motorController;
        private ISteeringBehavior _steeringBehavior;
        private Rigidbody _rigidbody;

        private NavMeshAgent _navMeshAgent;

        private void Awake()
        {
            _motorController = new WheelMotorController(_wheelRearLeft, _wheelRearRight);
            _steeringBehavior = new SimpleSteeringBehaviour();
            _rigidbody = GetComponent<Rigidbody>();
            _navMeshAgent = GetComponent<NavMeshAgent>();
            DisableNavMeshComponent();
        }

        private void Start()
        {
            if (_wayPointSystem != null)
            {
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
        }

        private void FixedUpdate()
        {
            if (_waypoints == null || _waypoints.Count == 0) return;

            _navMeshAgent.nextPosition = transform.position;

            if (!_navMeshAgent.pathPending && _navMeshAgent.remainingDistance <= _navMeshAgent.stoppingDistance)
            {
                SetNextWaypoint();
            }

            Vector3 desiredVelocity = _navMeshAgent.desiredVelocity;
            float steerAngle = _steeringBehavior.CalculateSteeringAngle(transform, transform.position + desiredVelocity, _maxSteerAngle);
            ApplySteering(steerAngle);

            ControlSpeed(desiredVelocity.magnitude);
        }

        private void SetNextWaypoint()
        {
            _currentWaypointIndex = (_currentWaypointIndex + 1) % _waypoints.Count;
            _navMeshAgent.SetDestination(_waypoints[_currentWaypointIndex]);
        }

        private void ApplySteering(float steerAngle)
        {
            _wheelFrontLeft.steerAngle = steerAngle;
            _wheelFrontRight.steerAngle = steerAngle;
        }

        private void ControlSpeed(float desiredSpeed)
        {
            float currentSpeed = _rigidbody.velocity.magnitude;

            if (currentSpeed < desiredSpeed)
            {
                _motorController.ReleaseBrakes();
                _motorController.ApplyMotorTorque(_maxMotorTorque);
            }
            else if (currentSpeed > desiredSpeed + 1f) // небольшая граница
            {
                _motorController.ApplyBrakeTorque(_maxBrakeTorque);
            }
            else
            {
                _motorController.ApplyMotorTorque(0f);
                _motorController.ReleaseBrakes();
            }
        }

        private void DisableNavMeshComponent()
        {
            _navMeshAgent.updatePosition = false;
            _navMeshAgent.updateRotation = false;

            _navMeshAgent.speed = _maxSpeed;
            _navMeshAgent.acceleration = 10f;
            _navMeshAgent.angularSpeed = 0f;
            _navMeshAgent.autoBraking = false;
        }

        private void OnDrawGizmos()
        {
            if (_waypoints != null && _waypoints.Count > 0)
            {
                Gizmos.color = Color.green;
                for (int i = 0; i < _waypoints.Count - 1; i++)
                {
                    Gizmos.DrawLine(_waypoints[i], _waypoints[i + 1]);
                }
                Gizmos.DrawLine(_waypoints[^1], _waypoints[0]);
            }
        }
    }
}

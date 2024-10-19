using UnityEngine;
using System.Collections.Generic;
using UnityEngine.AI;

namespace Client
{
    [RequireComponent(typeof(Rigidbody))]
    public class CarAiBehaviour : MonoBehaviour
    {
        [Header("Wheel Colliders")] [SerializeField]
        private WheelCollider _wheelFrontLeft;

        [SerializeField] private WheelCollider _wheelFrontRight;
        [SerializeField] private WheelCollider _wheelRearLeft;
        [SerializeField] private WheelCollider _wheelRearRight;

        [Header("Car Parameters")] [SerializeField]
        private float _maxMotorTorque = 150f;

        [SerializeField] private float _maxSteerAngle = 30f;
        [SerializeField] private float _maxSpeed = 100f;

        [Header("Braking Parameters")] [SerializeField]
        private float _brakingDistance = 10f;

        [SerializeField] private float _maxBrakeTorque = 300f;

        [Header("Point")] [SerializeField] private float _point;

        private IPathfinder _pathfinder;
        private ISteeringBehavior _steeringBehavior;
        private IMotorController _motorController;
        private Rigidbody _rigidbody;
        private NavMeshAgent _navMeshAgent;

        private List<Vector3> _pathCorners;
        private int _currentCornerIndex = 0;

        private const float Force = 3.6f;

        private void Awake()
        {
            _pathfinder = new AiPathfinder();
            _steeringBehavior = new SimpleSteeringBehaviour();
            _motorController = new WheelMotorController(_wheelRearLeft, _wheelRearRight);

            _rigidbody = GetComponent<Rigidbody>();
            _navMeshAgent = GetComponent<NavMeshAgent>();
            DisableNavMeshComponent();
        }

        private void Start()
        {
            /*if (_pointB == null)
            {
                Debug.LogError("Destination is not assigned.");
                enabled = false;
                return;
            }

            CalculatePath();*/
        }

        private void FixedUpdate()
        {
            _navMeshAgent.nextPosition = transform.position;

            Vector3 forwardPoint = transform.position + transform.forward * _point;

            NavMeshHit hit;
            if (NavMesh.SamplePosition(forwardPoint, out hit, 10f, NavMesh.AllAreas))
            {
                _navMeshAgent.SetDestination(hit.position);
            }

            Vector3 desiredVelocity = _navMeshAgent.desiredVelocity;

            float steerAngle =
                _steeringBehavior.CalculateSteeringAngle(transform, transform.position + desiredVelocity,
                    _maxSteerAngle);
            ApplySteering(steerAngle);

            ControlSpeed(desiredVelocity.magnitude);
        }


        /*private void CalculatePath()
        {
            _pathCorners = _pathfinder.CalculatePath(transform.position, _pointB.position);
            if (_pathCorners == null || _pathCorners.Count == 0)
            {
                Debug.LogError("Failed to calculate path.");
                enabled = false;
            }
        }*/

        private void ApplySteering(float steerAngle)
        {
            _wheelFrontLeft.steerAngle = steerAngle;
            _wheelFrontRight.steerAngle = steerAngle;
        }

        private void ControlSpeed(float desiredSpeed)
        {
            var currentSpeed = _rigidbody.velocity.magnitude; // м/с

            if (currentSpeed < desiredSpeed)
            {
                _motorController.ReleaseBrakes();
                _motorController.ApplyMotorTorque(_maxMotorTorque);
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

            _navMeshAgent.speed = _maxSpeed / Force;
            _navMeshAgent.acceleration = 10f;
            _navMeshAgent.angularSpeed = 0f;
            _navMeshAgent.autoBraking = false;
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
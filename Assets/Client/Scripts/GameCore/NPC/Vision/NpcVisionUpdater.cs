using System;
using System.Collections.Generic;
using CustomTools.Updater;
using UnityEngine;

namespace Client
{
    public class NpcVisionUpdater : IUpdateMono
    {
        private readonly CarAiBehaviour _carAiBehaviour;
        private readonly Transform _transform;
        private readonly float _visionLength;
        private float _sensorTurnAmount;
        private bool _obstacleInPath;
        private float _obstacleAngle;
        private readonly string _ignoreMask;

        private readonly VisionItem[] _visionItems;

        public NpcVisionUpdater(CarAiBehaviour carAiBehaviour, Transform transform, float visionLength,
            string ignoreMask,
            VisionItem[] visionItems)
        {
            _carAiBehaviour = carAiBehaviour;
            _transform = transform;
            _visionLength = visionLength;
            _ignoreMask = ignoreMask;
            _visionItems = visionItems;
        }

        private bool IsObstacleInPath()
        {
            for (int i = 0; i < _visionItems.Length; i++)
            {
                if (Math.Abs(_visionItems[i].Weight - 1) < 0.01f)
                {
                    return true;
                }
            }

            return false;
        }

        private float VisionValue(IReadOnlyList<VisionItem> sensors)
        {
            float sensorValue = 0;
            for (var i = 0; i < sensors.Count; i++)
            {
                sensorValue += sensors[i].Weight * sensors[i].Direction;
            }

            return sensorValue;
        }

        public void Tick()
        {
            PerformRaycastCheck();
        }

        private void PerformRaycastCheck()
        {
            foreach (var vision in _visionItems)
            {
                if (vision.Point.localPosition.x == 0)
                {
                    vision.Direction = 0;
                }
                else
                {
                    vision.Direction = vision.Point.localPosition.x /
                                       Mathf.Abs(vision.Point.localPosition.x);
                }

                if (Physics.Raycast(vision.Point.position, vision.Point.forward, out vision.Hit,
                        _visionLength))
                {
                    if (vision.Hit.collider.CompareTag(_ignoreMask))
                    {
                        vision.Weight = 1;
                    }
                    
                    vision.Weight = 0;
                    Debug.DrawLine(vision.Point.position, vision.Hit.point, Color.red);
                }
                else
                {
                    vision.Weight = 1;
                }
            }

            _obstacleInPath = IsObstacleInPath();

            _sensorTurnAmount = VisionValue(_visionItems);

            if (_sensorTurnAmount == 0 && _obstacleInPath)
            {
                _obstacleAngle = Vector3.Dot(_visionItems[1].Hit.normal, _transform.right);

                if (_obstacleAngle > 0)
                {
                    _carAiBehaviour.TurnValue = -1;
                }
                if (_obstacleAngle < 0)
                {
                    _carAiBehaviour.TurnValue = 1;
                }
            }
            else
            {
                _carAiBehaviour.TurnValue = Mathf.Sign(_sensorTurnAmount);
            }
        }
    }
}
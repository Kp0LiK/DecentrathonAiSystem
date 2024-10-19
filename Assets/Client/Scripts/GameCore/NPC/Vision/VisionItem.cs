using System;
using UnityEngine;

namespace Client
{
    [Serializable]
    public class VisionItem
    {
        [field: SerializeField] public Transform Point { get; set; }
        public float Weight { get; set; }
        public float Direction { get; set; }
        public RaycastHit Hit;
    }
}
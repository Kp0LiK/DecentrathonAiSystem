using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Client
{
    public class NpcStater : MonoBehaviour, INPCStateSwitcher
    {
        private List<BaseNPCState> _states;

        private BaseNPCState _currentState;

        private void Awake()
        {
            _states = new List<BaseNPCState>
            {
                new NPCFollowPathState(this),
                new NPCObstacleDetectState(this)
            };
            
            _currentState = _states[0];
        }


        public void SwitchState<T>() where T : BaseNPCState
        {
            var state = _states.FirstOrDefault(s => s is T);

            if (ReferenceEquals(state, null))
            {
                Debug.LogError($"[Npc State] Can't find {typeof(T)}");
                return;
            }

            _currentState.EndState();
            _currentState = state;

            _currentState.StartState();
        }
    }
}
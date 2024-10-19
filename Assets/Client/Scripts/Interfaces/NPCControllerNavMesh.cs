using UnityEngine;
using UnityEngine.AI;

namespace Client
{
    public class NPCControllerNavMesh : MonoBehaviour
    {
        [SerializeField] private NavMeshAgent agent;
        [SerializeField] private Transform trackStartPoint; // Начальная точка трека
        [SerializeField] private float distanceThreshold = 1f; // Порог для перехода к следующей точке

        private NavMeshPath path;
        private int currentCorner = 0;

        private void Start()
        {
            path = new NavMeshPath();
            agent.SetDestination(trackStartPoint.position); // Начальная точка маршрута
            InvokeRepeating(nameof(CalculatePath), 0, 1f); // Перерасчет пути раз в секунду
        }

        private void CalculatePath()
        {
            // Вычисляем маршрут по всей дороге от текущей позиции до точки старта трека
            if (NavMesh.CalculatePath(transform.position, trackStartPoint.position, NavMesh.AllAreas, path))
            {
                if (path.corners.Length > 1)
                {
                    MoveAlongPath();
                }
            }
        }

        private void MoveAlongPath()
        {
            // Проверка, если агент достиг текущей угловой точки
            if (Vector3.Distance(transform.position, path.corners[currentCorner]) < distanceThreshold)
            {
                currentCorner++;
            
                // Если достигли последней точки, начинаем заново
                if (currentCorner >= path.corners.Length)
                {
                    currentCorner = 0;
                }
            }

            // Задаем следующее направление движения по NavMesh
            agent.SetDestination(path.corners[currentCorner]);
        }

        private void OnDrawGizmos()
        {
            // Рисуем путь для визуализации маршрута
            if (path != null && path.corners.Length > 1)
            {
                for (int i = 0; i < path.corners.Length - 1; i++)
                {
                    Gizmos.color = Color.red;
                    Gizmos.DrawLine(path.corners[i], path.corners[i + 1]);
                }
            }
        }
    }
}
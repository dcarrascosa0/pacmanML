using UnityEngine;
using UnityEngine.AI;

public class Pinky : MonoBehaviour
{
    public MLAgent pacmanAgent;
    private NavMeshAgent navMeshAgent;

    void Start()
    {
        navMeshAgent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        Vector3 pacmanDirection = new Vector3(pacmanAgent.directionX, 0, pacmanAgent.directionZ).normalized;
        Vector3 offset = pacmanDirection * 4;
        navMeshAgent.destination = pacmanAgent.transform.localPosition + offset;
    }
}

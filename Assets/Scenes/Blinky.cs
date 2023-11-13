using UnityEngine;
using UnityEngine.AI;

public class Blinky : MonoBehaviour
{
    public MLAgent pacmanAgent;
    private NavMeshAgent navMeshAgent;

    void Start()
    {
        navMeshAgent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        navMeshAgent.destination = pacmanAgent.transform.localPosition;
    }
}

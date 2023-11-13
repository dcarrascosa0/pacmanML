using UnityEngine;
using UnityEngine.AI;

public class Clyde : MonoBehaviour
{
    public MLAgent pacmanAgent;
    public Vector3 farAwayPoint;
    private NavMeshAgent navMeshAgent;

    void Start()
    {
        navMeshAgent = GetComponent<NavMeshAgent>();
    }

    void Update()
    {
        float distance = Vector3.Distance(transform.localPosition, pacmanAgent.transform.localPosition);
        if (distance < 8)
        {
            navMeshAgent.destination = farAwayPoint;
        }
        else
        {
            navMeshAgent.destination = pacmanAgent.transform.localPosition;
        }
    }
}

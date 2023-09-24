using UnityEngine;
using UnityEngine.AI;

public class Inky : MonoBehaviour
{
    public MLAgent pacmanAgent;
    private NavMeshAgent navMeshAgent;
    private float timeSinceLastUpdate;
    public float updateInterval = 5.0f;

    void Start()
    {
        navMeshAgent = GetComponent<NavMeshAgent>();
        timeSinceLastUpdate = 0;
    }

    void Update()
    {
        timeSinceLastUpdate += Time.deltaTime;
        if (navMeshAgent.remainingDistance < 0.5f || timeSinceLastUpdate > updateInterval)
        {
            Vector3 randomOffset = new Vector3(Random.Range(-10, 10), 0, Random.Range(-10, 10));
            navMeshAgent.destination = pacmanAgent.transform.localPosition + randomOffset;
            timeSinceLastUpdate = 0;
        }
    }
}

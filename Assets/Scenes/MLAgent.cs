using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine.AI;
using System.Linq;


public class MLAgent : Agent
{
    [SerializeField] private float speed = 1.0f;
    [SerializeField] private Material winMaterial, loseMaterial;
    [SerializeField] private MeshRenderer floor;
    [SerializeField] private int numSteps;

    private Vector3 lastPosition;
    private bool isMoving, canAttack;
    public float directionX, directionZ;
    private Vector3 manualDirection = Vector3.zero;
    private Goal[] goalObjects;
    private GhostBehaviour[] ghostObjects;
    private BehaviorParameters behaviorParameters;
    private float moveThreshold = 0.001f;

    private Goal closestGoal;
    private GhostBehaviour closestGhost;

    private float lastCalculatedClosestGoalDistance = float.MaxValue;
    private float lastCalculatedClosestGhostDistance = float.MaxValue;

    private NavMeshAgent navMeshAgent;

    private Vector3 startPosition;

    private int numPallets;

    private float lastPalletEatenTime;

    private PalletsBehaviour[] palletObjects;

    private Quaternion startRotation;

    private int pelletsEaten = 0;

    private int currentStep = 0;

    public int currentLesson = 0;
    public float[] lessonPalletPercentage = new float[] { 0, 1f, 0, 2f, 0.4f, 0.6f, 0.8f, 1.0f };
    private float previousPalletPercentage = -1.0f;
    private float numPalletsToActivate = 1;

    public PalletsBehaviour closestPallet;

    public List<Vector3> waypoints;
    public NavMeshPath path;

    private List<Vector3> positionHistory = new List<Vector3>();
    private const int penaltyThreshold = 5;
    private float distanceThreshold = 0.05f;  // Adjust this value based on what you consider a "small" distance






    void Start()
    {
        Transform roomTransform = this.transform.parent;
        InitializeTransformsBasedOnParent(roomTransform);
        InitializeComponents();

    }

    private void InitializeTransformsBasedOnParent(Transform roomTransform)
    {
        Transform ghostsParent = roomTransform.Find("Ghosts");
        Transform goalsParent = roomTransform.Find("Goals");
        Transform palletsParent = roomTransform.Find("Pallets");

        numPallets = palletsParent.childCount;
        goalObjects = goalsParent.GetComponentsInChildren<Goal>();
        palletObjects = palletsParent.GetComponentsInChildren<PalletsBehaviour>();
    }

    private void InitializeComponents()
    {
        behaviorParameters = GetComponent<BehaviorParameters>();
        lastPosition = transform.localPosition;
        navMeshAgent = GetComponent<NavMeshAgent>();
        startPosition = transform.localPosition;
        startRotation = transform.rotation;

        waypoints = new List<Vector3>();
        path = new NavMeshPath();
    }

    public override void OnEpisodeBegin()
    {
        Transform roomTransform = this.transform.parent;
        foreach (Goal goal in goalObjects)
        {
            goal.gameObject.SetActive(true);
        }

        transform.localPosition = startPosition;


        float currentPalletPercentage = Academy.Instance.EnvironmentParameters.GetWithDefault("pallet_percentage", 1.0f);

        if (previousPalletPercentage != currentPalletPercentage)
        {
            Debug.Log($"Current Pallet Percentage: {currentPalletPercentage}");
            previousPalletPercentage = currentPalletPercentage;
        }

        numPalletsToActivate = (float) Mathf.RoundToInt(numPallets * currentPalletPercentage);

        ActivatePallets(numPalletsToActivate);

        //closestGoal = GetClosestEntity(goalObjects);
        closestPallet = GetClosestEntity(palletObjects);

        lastPalletEatenTime = Time.time;
        pelletsEaten = 0;
        currentStep = 0;
        positionHistory.Clear();

    }


    public override void CollectObservations(VectorSensor sensor)
    {
        // Add relative position to closest goal and pallet

        // Add time since last pallet was eaten
        sensor.AddObservation(Time.time - lastPalletEatenTime);

        closestPallet = GetClosestEntity(palletObjects);

        if (closestPallet != null)
        {
            // Add path to closest pallet using waypoints
            AddPathToObservations(sensor, closestPallet.transform.position, 1);

            sensor.AddObservation(transform.localPosition);
            float palletDistance = CalculateNavMeshPathLength(transform.localPosition, closestPallet.transform.localPosition);
            sensor.AddObservation(palletDistance == -1 ? -1 : palletDistance);
        }
        else
        {
            // Handle case when closestPallet is null, maybe by adding default values
            // This is just an example; you can use other default values as needed.
            sensor.AddObservation(-1);
            sensor.AddObservation(transform.localPosition);
            sensor.AddObservation(-1);
        }

        sensor.AddObservation(numPalletsToActivate - pelletsEaten);
    }




    private void AddPathToObservations(VectorSensor sensor, Vector3 targetWorldPosition, int maxWaypoints)
    {
        waypoints.Clear();
        path.ClearCorners();
        waypoints = CalculateNavMeshPath(transform.position, targetWorldPosition);
        Transform roomTransform = this.transform.parent;


        // Convert world coordinates to local coordinates
        for (int i = 0; i < waypoints.Count; i++)
        {
            waypoints[i] = roomTransform.InverseTransformPoint(waypoints[i]);
        }

        // Limit the waypoints to maxWaypoints
        int actualWaypointCount = Mathf.Min(waypoints.Count, maxWaypoints);

        for (int i = 0; i < actualWaypointCount; i++)
        {
            sensor.AddObservation(waypoints[i]);
        }

        // Pad the rest with zeros
        for (int i = actualWaypointCount; i < maxWaypoints; i++)
        {
            sensor.AddObservation(Vector3.zero);
        }
    }



    public float CalculateNavMeshPathLength(Vector3 start, Vector3 end)
    {
        NavMeshPath currentPath = new NavMeshPath();
        if (NavMesh.CalculatePath(start, end, NavMesh.AllAreas, currentPath))
        {
            float length = 0;
            for (int i = 0; i < currentPath.corners.Length - 1; i++)
            {
                length += Vector3.Distance(currentPath.corners[i], currentPath.corners[i + 1]);
            }
            return length;
        }
        return -1;  // Changed from float.MaxValue to -1
    }


    public List<Vector3> CalculateNavMeshPath(Vector3 worldStart, Vector3 worldEnd)
    {
        if (NavMesh.CalculatePath(worldStart, worldEnd, NavMesh.AllAreas, path))
        {
            waypoints.AddRange(path.corners);
        }
        return waypoints;
    }


    private T GetClosestEntity<T>(T[] entities) where T : MonoBehaviour
    {
        return entities
            .Where(e => e.gameObject.activeSelf)
            .OrderBy(e => CalculateNavMeshPathLength(transform.localPosition, e.transform.localPosition))
            .FirstOrDefault();
    }



    public override void OnActionReceived(ActionBuffers actions)
    {
        HandleMovement(actions);
        Vector3 newPosition = transform.localPosition;
        //UpdatePosition(newPosition);
        //CheckWaypointProximity();
        HandleRewards();

        AddReward(-10.0f/MaxStep);
        currentStep++;
        ;
        if (currentStep >= MaxStep || Time.time - lastPalletEatenTime > 30)
        {
            float penalty = (numPalletsToActivate - pelletsEaten) * -0.5f; // Adjust the penalty factor as needed
            AddReward(penalty);
            Debug.Log("Ending episode due to max steps reached or 30 seconds without eating a pallet.");
            EndEpisode();
        }
    }

    private void UpdatePosition(Vector3 newPosition)
    {
        if (IsStuck(newPosition))
        {
            AddReward(-10f);
            Debug.Log("Ending episode due to Loop movment");
            EndEpisode();
        }
        else
        {
            positionHistory.Add(newPosition);

            if (positionHistory.Count > penaltyThreshold)
                positionHistory.RemoveAt(0);  // Keep only the recent positions
        }
        
    }

    private bool IsStuck(Vector3 newPosition)
    {
        

        for (int i = 0; i < positionHistory.Count; i++)
        {
            Vector3 current = positionHistory[i];

            if (Vector3.Distance(current, newPosition) < distanceThreshold)
                return true;
        }

        return false;
    }


    private void HandleMovement(ActionBuffers actions)
    {
        directionX = actions.ContinuousActions[0];
        directionZ = actions.ContinuousActions[1];

        Vector3 move = transform.right * directionX + transform.forward * directionZ;
        move.Normalize();

        Vector3 futurePosition = transform.localPosition + move * speed * Time.deltaTime;

        // Ray length adjusted, and debug ray added
        float rayLength = speed * Time.deltaTime;
        Debug.DrawRay(transform.localPosition, move * rayLength, Color.red);

        if (Physics.Raycast(transform.localPosition, move, rayLength, LayerMask.GetMask("Wall")))
        {
            AddReward(-0.01f);
        }
        else
        {
            transform.localPosition = futurePosition;
            lastPosition = transform.localPosition;
        }

        transform.rotation = startRotation;
    }

    private void CheckWaypointProximity()
    {
        if (waypoints.Count > 0)
        {
            float distanceToWaypoint = Vector3.Distance(transform.localPosition, waypoints[0]);

            if (distanceToWaypoint < 5) // e.g., 0.5f
            {
                AddReward(5 - distanceToWaypoint * 0.0000001f); // Inverse of remapped value for closer distances
            }
            else
            {
                AddReward(-distanceToWaypoint * 0.00000001f);
            }
        }

    }





    private void HandleRewards()
    {
        if(closestGoal!=null)
        {
            UpdateRewardForDistanceToEntity(closestGoal.transform.localPosition, ref lastCalculatedClosestGoalDistance);
        }
        //UpdateRewardForDistanceToEntity(closestGhost.transform.position, ref lastCalculatedClosestGhostDistance);
    }

    private void UpdateRewardForDistanceToEntity(Vector3 entityPosition, ref float lastCalculatedDistance)
    {
        float currentDistance = CalculateNavMeshPathLength(transform.localPosition, entityPosition);
        if (currentDistance != -1 && currentDistance < lastCalculatedDistance)  
        {
            AddReward(0.001f);
        }
        else
        {
            AddReward(-0.001f);
        }
        lastCalculatedDistance = currentDistance;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        continuousActions[0] = Input.GetAxisRaw("Horizontal");
        continuousActions[1] = Input.GetAxisRaw("Vertical");
    }

    void Update()
    {
        if (!IsUsingHeuristic()) return;

        float horizontalInput = Input.GetAxisRaw("Horizontal");
        float verticalInput = Input.GetAxisRaw("Vertical");
        manualDirection = new Vector3(horizontalInput, 0, verticalInput).normalized;
        IsMoving();
    }

    public bool IsMoving()
    {
        isMoving = Vector3.Distance(transform.localPosition, lastPosition) > moveThreshold;
        if (!isMoving)
        {
            AddReward(-0.001f); // Penalize for idling
        }
        return isMoving;
    }


    private bool IsUsingHeuristic()
    {
        return behaviorParameters.BehaviorType == BehaviorType.HeuristicOnly;
    }

    private void DeactivateAttack()
    {
        canAttack = false;
    }



    private void OnCollisionEnter(Collision other)
    {

        if (other.gameObject.TryGetComponent<Goal>(out Goal goalC))
        {
            AddReward(1.0f);
            canAttack = true;
            /*foreach (GhostBehaviour ghost in ghostObjects)
            {
                ghost.EnterFrightenedMode();
            }
            */
            CancelInvoke("DeactivateAttack");
            Invoke("DeactivateAttack", 10f);
            goalC.gameObject.SetActive(false);
            closestGoal = GetClosestEntity(goalObjects);


        }
        else if (other.gameObject.TryGetComponent<Wall>(out Wall wall))
        {
            AddReward(-0.01f);
        }
        else if (other.gameObject.TryGetComponent<GhostBehaviour>(out GhostBehaviour ghost))
        {
            if (canAttack)
            {
                float attackReward = 0.5f + 0.1f ;  // Increase reward with each consecutive pallet eaten
                AddReward(attackReward);
            }
            else
            {
                AddReward(-1f);
                Debug.Log("Ending episode due to ghost collision without canAttack.");
                EndEpisode();
            }

        }

    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("pallets"))
        {
            float timeDiff = Time.time - lastPalletEatenTime;

            // Base reward for eating a pellet
            float baseReward = 1.0f / numPalletsToActivate;

            // Time-based reward
            float timeBasedReward = 1 - Mathf.Clamp01(timeDiff / 30);  // Reward decreases as time increases

            float totalReward = baseReward + timeBasedReward;

            AddReward(totalReward);

            lastPalletEatenTime = Time.time;
            other.gameObject.SetActive(false);

            pelletsEaten++;

            // Check if all pellets are eaten
            if (pelletsEaten >= numPalletsToActivate)
            {
                AddReward(30.0f);  // Give positive reward
                Debug.Log("Ending episode because all pellets are eaten.");
                EndEpisode();     // Restart the episode
            }
            closestPallet = GetClosestEntity(palletObjects);
        }
    }

    private void ActivatePallets(float nPalletsToActivate)
    {
        // Ensure you're shuffling or selecting pallets randomly if that's desired
        List<PalletsBehaviour> shuffledPallets = palletObjects.OrderBy(p => UnityEngine.Random.value).ToList();

        for (int i = 0; i < numPallets; i++)
        {
            if (i < nPalletsToActivate)
                shuffledPallets[i].gameObject.SetActive(true);
            else
                shuffledPallets[i].gameObject.SetActive(false);
        }
    }

    void OnDrawGizmos()
    {
        if (waypoints.Count > 0)
        {
            Transform roomTransform = this.transform.parent;
            Gizmos.color = Color.red;

            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                Vector3 worldPoint1 = roomTransform.TransformPoint(waypoints[i]);
                Vector3 worldPoint2 = roomTransform.TransformPoint(waypoints[i + 1]);
                Gizmos.DrawLine(worldPoint1, worldPoint2);
            }
        }
    }


}

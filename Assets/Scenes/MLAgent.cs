using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine.AI;
using System.Linq;
using System.Runtime.CompilerServices;

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
    private List<GameObject> goalObjects;
    private List<GameObject> ghostObjects;
    private BehaviorParameters behaviorParameters;
    private float moveThreshold = 0.001f;


    private float lastCalculatedClosestGoalDistance = float.MaxValue;
    private float lastCalculatedClosestGhostDistance = float.MaxValue;

    private NavMeshAgent navMeshAgent;

    private Vector3 startPosition;

    private int numPallets;

    private float lastPalletEatenTime;

    private List<GameObject> palletObjects;

    private Quaternion startRotation;

    private int pelletsEaten = 0;

    private int currentStep = 0;

    public int currentLesson = 0;
    public float[] lessonPalletPercentage = new float[] { 0, 1f, 0, 2f, 0.4f, 0.6f, 0.8f, 1.0f };
    private float previousPalletPercentage = -1.0f;
    private float numPalletsToActivate = 1;
    private float numGhostsToActivate = 0;



    public List<Vector3> waypoints;
    

    public NavMeshPath path;

    private List<Vector3> positionHistory = new List<Vector3>();
    private const int penaltyThreshold = 5;
    private float distanceThreshold = 0.05f;  // Adjust this value based on what you consider a "small" distance
    private float previousDistancePallet = float.MaxValue;
    private float previousDistanceGoal = float.MaxValue;
    private float previousDistanceGhost = float.MaxValue;

    private Rigidbody rb;  // Rigidbody component
    private float lastGoalEatenTime;

    private bool hasEatenGhost;
    private float maxDistanceToWaypoint = 0f;
    public GameObject optimalTarget;
    private float previousPathLength = float.MaxValue;
    private Vector3 lastRewardedWaypoint;
    Vector2 directionToFirstWaypoint;
    float distanceToWaypoint;
    private float countdownTimer = 0f;
    private LessonManager lessonManager;



    void Start()
    {
        Transform roomTransform = this.transform.parent;
        InitializeTransformsBasedOnParent(roomTransform);
        InitializeComponents();
        rb = GetComponent<Rigidbody>();

    }

    private void InitializeTransformsBasedOnParent(Transform roomTransform)
    {
        Transform ghostsParent = roomTransform.Find("Ghosts");
        Transform goalsParent = roomTransform.Find("Goals");
        Transform palletsParent = roomTransform.Find("Pallets");

        ghostObjects = new List<GameObject>(Array.ConvertAll(ghostsParent.GetComponentsInChildren<GhostBehaviour>(), item => item.gameObject));
        goalObjects = new List<GameObject>(Array.ConvertAll(goalsParent.GetComponentsInChildren<Goal>(), item => item.gameObject));
        palletObjects = new List<GameObject>(Array.ConvertAll(palletsParent.GetComponentsInChildren<PalletsBehaviour>(), item => item.gameObject));

        numPallets = palletsParent.childCount;

        lessonManager = FindObjectOfType<LessonManager>();
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
        
        // Your existing code
        Transform roomTransform = this.transform.parent;
        foreach (GameObject goal in goalObjects)
        {
            goal.gameObject.SetActive(true);
        }

        // Use LessonManager to get the current combined lesson
        LessonManager.LessonData lessonData = lessonManager.GetCurrentCombinedValue();
        numGhostsToActivate = lessonData.ghostValue;

        // Reset and activate only 'currentNumGhosts' ghosts
        for (int i = 0; i < ghostObjects.Count; i++)
        {
            if (i < numGhostsToActivate)
            {
                ghostObjects[i].gameObject.SetActive(true);
                ghostObjects[i].GetComponent<GhostBehaviour>().Reset();
            }
            else
            {
                ghostObjects[i].gameObject.SetActive(false);
            }
        }

        transform.localPosition = startPosition;

        numPalletsToActivate = (float)Mathf.RoundToInt(numPallets * lessonData.palletValue);
        ActivatePallets(numPalletsToActivate);

        lastPalletEatenTime = Time.time;
        pelletsEaten = 0;
        currentStep = 0;
        positionHistory.Clear();
        hasEatenGhost = false;


}



public override void CollectObservations(VectorSensor sensor)
    {
        optimalTarget = FindOptimalPath(transform.position);
        Transform roomTransform = this.transform.parent;
        waypoints.Clear();
        float maxTimeBetweenPallets = 30f;

        if (optimalTarget != null)
        {
            NavMeshPath path = new NavMeshPath();
            NavMesh.CalculatePath(transform.position, optimalTarget.transform.position, NavMesh.AllAreas, path);

            waypoints = path.corners.Select(point => roomTransform.InverseTransformPoint(point)).ToList();

            if (waypoints.Count > 1)
            {
                directionToFirstWaypoint = new Vector2(waypoints[1].x, waypoints[1].z) - new Vector2(transform.localPosition.x, transform.localPosition.z);

                distanceToWaypoint = directionToFirstWaypoint.magnitude;
                maxDistanceToWaypoint = Mathf.Max(maxDistanceToWaypoint, distanceToWaypoint);

                // Normalize
                sensor.AddObservation(directionToFirstWaypoint.normalized);
                sensor.AddObservation( GetPathLength(path));
            }
        }
        else
        {
            sensor.AddObservation(Vector2.zero);
            sensor.AddObservation(0.0f);
        }

        if(ghostObjects.Count > 0)
        {
            foreach (var ghostObject in ghostObjects){
                if (ghostObject.activeInHierarchy)
                {
                    NavMeshPath pathGhost = new NavMeshPath();
                    NavMesh.CalculatePath(transform.position, ghostObject.transform.position, NavMesh.AllAreas, pathGhost);
                    float pathLength = GetPathLength(pathGhost);
                    sensor.AddObservation(pathLength);
                }
                else
                {
                    sensor.AddObservation(-1f);
                }
            }

        }
        
        sensor.AddObservation(countdownTimer);
        sensor.AddObservation(canAttack ? 1f : 0f);
        sensor.AddObservation((float)(numPalletsToActivate - pelletsEaten) / numPalletsToActivate);
        sensor.AddObservation((Time.time - lastPalletEatenTime) / maxTimeBetweenPallets);
        sensor.AddObservation(new Vector2(transform.localPosition.x, transform.localPosition.z));
        sensor.AddObservation(new Vector2(rb.velocity.x, rb.velocity.z).normalized);
    }


    public float GetPathLength(NavMeshPath path)
    {
        float length = 0.0f;
        for (int i = 1; i < path.corners.Length; ++i)
        {
            length += Vector3.Distance(path.corners[i - 1], path.corners[i]);
        }
        return length;
    }

    public GameObject FindOptimalPath(Vector3 startPos)
    {
        NavMeshPath path = new NavMeshPath();
        float bestCost = Mathf.Infinity;
        GameObject optimalTarget = null;

        // Step 1: Get nearest targets
        var nearestPallets = palletObjects
            .Where(x => x.activeInHierarchy)
            .OrderBy(x => Vector3.Distance(startPos, x.transform.position))
            .Take(10);

        var nearestGoals = goalObjects
            .Where(x => x.activeInHierarchy)
            .OrderBy(x => Vector3.Distance(startPos, x.transform.position))
            .Take(1);

        var nearestGhosts = ghostObjects
            .Where(x => x.activeInHierarchy)
            .OrderBy(x => Vector3.Distance(startPos, x.transform.position))
            .Take(4);

        List<GameObject> nearestTargets = new List<GameObject>(nearestPallets);
        nearestTargets.AddRange(nearestGoals);
        nearestTargets.AddRange(nearestGhosts);

        // Step 2: Calculate path cost and penalties for each target
        foreach (GameObject target in nearestTargets)
        {
            if (NavMesh.CalculatePath(startPos, target.transform.position, NavMesh.AllAreas, path))
            {
                float baseCost = GetPathLength(path);

                float penalty = 0;
                foreach (GameObject ghost in ghostObjects)
                {
                    if (IsGhostInPath(path, ghost.transform.position))
                    {
                        penalty += 1000;
                    }
                }

                if (!canAttack)
                {
                    float closestGhostDist = float.MaxValue;

                    foreach (GameObject ghost in ghostObjects)
                    {
                        float dist = Vector3.Distance(target.transform.position, ghost.transform.position);
                        closestGhostDist = Mathf.Min(closestGhostDist, dist);
                    }

                    // Use inverse relationship for penalty: farther distance, smaller penalty; closer distance, greater penalty.
                    if (closestGhostDist != float.MaxValue)
                    {
                        penalty += 30 / (closestGhostDist + 1);  // +1 to avoid division by zero
                    }

                    // Give priority to the goal when not in attack mode and a ghost is nearby
                    if (closestGhostDist < 4f && goalObjects.Contains(target))
                    {
                        baseCost *= 0.2f;
                    }
                }

                if (goalObjects.Contains(target))
                {
                    baseCost *= 0.5f;
                }

                if (ghostObjects.Contains(target))
                {
                    baseCost *= canAttack ? 0.1f : 5f;
                }

                float totalCost = baseCost + penalty;
                if (totalCost < bestCost)
                {
                    bestCost = totalCost;
                    optimalTarget = target;
                }
            }
        }

        return optimalTarget;
    }


        public bool IsGhostInPath(NavMeshPath path, Vector3 ghostPosition)
    {
        for (int i = 1; i < path.corners.Length; ++i)
        {
            // Check if ghost is close to any segment of the path
            if (IsPointNearSegment(path.corners[i - 1], path.corners[i], ghostPosition, 1f))
            {
                return true;
            }
        }
        return false;
    }

    public bool IsPointNearSegment(Vector3 a, Vector3 b, Vector3 point, float threshold)
    {
        float closestDistance = Mathf.Clamp(Vector3.Dot(point - a, b - a) / Vector3.Magnitude(b - a), 0, 1);
        float distance = Vector3.Magnitude(a + closestDistance * (b - a) - point);

        return distance <= threshold;
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (countdownTimer > 0)
        {
            countdownTimer -= Time.deltaTime;
        }

        HandleMovement(actions);
        Vector3 newPosition = transform.localPosition;

        AddReward(-0.3f/(float)MaxStep);
        currentStep++;
        
        if (currentStep >= MaxStep || Time.time - lastPalletEatenTime > 30)
        {
            AddReward((-(float)(numPalletsToActivate - pelletsEaten) / (float)numPalletsToActivate));
            EndEpisode();
        }

        RewardForCosineSimilarity();
        RewardForDistanceDecrease();
        RewardForReachingWaypoint();
    }


    private void RewardForReachingWaypoint()
    {
        if (waypoints.Count > 1)
        {
            Vector3 currentWaypoint = new Vector3(waypoints[1].x, waypoints[1].y, waypoints[1].z);
            if (Vector3.Distance(transform.localPosition, currentWaypoint) < 0.1f)
            {
                if (currentWaypoint != lastRewardedWaypoint)
                {
                    AddReward(0.05f);  // Large positive reward
                    lastRewardedWaypoint = currentWaypoint;
                }
            }
        }
    }

    private void RewardForCosineSimilarity()
    {
        if (directionToFirstWaypoint!=null)
        {
            Vector2 actualDirection = new Vector2(rb.velocity.x, rb.velocity.z).normalized;
            float rewardCosine = Vector2.Dot(actualDirection, directionToFirstWaypoint.normalized) * 3f;
            AddReward(rewardCosine / (float)MaxStep);
        }
    }

    private void RewardForDistanceDecrease()
    {
        

        if (distanceToWaypoint < previousPathLength)
        {
            AddReward(0.2f / (float)MaxStep);  // Positive reward for getting closer
        }
        else
        {
            AddReward(-0.2f / (float)MaxStep); // Negative reward for getting farther away
        }

        previousPathLength = distanceToWaypoint;
    }

    private void HandleMovement(ActionBuffers actions)
    {
        directionX = actions.ContinuousActions[0];
        directionZ = actions.ContinuousActions[1];

        Vector3 move = transform.right * directionX + transform.forward * directionZ;
        move.Normalize();

        // Calculate the future velocity
        Vector3 futureVelocity = move * speed;
        futureVelocity.y = 0.0f;

        // Apply movement as velocity change
        rb.AddForce(futureVelocity - rb.velocity, ForceMode.VelocityChange);

        transform.rotation = startRotation;
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
    }


    private bool IsUsingHeuristic()
    {
        return behaviorParameters.BehaviorType == BehaviorType.HeuristicOnly;
    }

    private void DeactivateAttack()
    {
        if (!hasEatenGhost)
        {
            AddReward(-0.01f);  // Negative reward for not eating a ghost within the activation time
        }
        canAttack = false;
        hasEatenGhost = false;
        // Cache these somewhere earlier
        List<GhostBehaviour> ghostBehaviours = ghostObjects.Select(go => go.GetComponent<GhostBehaviour>()).ToList();

        foreach (var ghostBehaviour in ghostBehaviours)
        {
            ghostBehaviour.EndFrightenedMode();
        }
    }
    public bool IsMoving()
    {
        return GetComponent<Rigidbody>().velocity != Vector3.zero;
    }



    private void OnCollisionEnter(Collision other)
    {

        if (other.gameObject.TryGetComponent<Goal>(out Goal goalC))
        {
            AddReward(0.15f/(float)goalObjects.Count());
            canAttack = true;
            List<GhostBehaviour> ghostBehaviours = ghostObjects.Select(go => go.GetComponent<GhostBehaviour>()).ToList();

            foreach (var ghostBehaviour in ghostBehaviours)
            {
                ghostBehaviour.EnterFrightenedMode();
            }
            countdownTimer = 10f;
            CancelInvoke("DeactivateAttack");
            Invoke("DeactivateAttack", 10f);
            goalC.gameObject.SetActive(false);
            lastGoalEatenTime = Time.time;



        }
        else if (other.gameObject.TryGetComponent<Wall>(out Wall wall))
        {
            AddReward(-0.01f / (float)MaxStep);
        }
        else if (other.gameObject.TryGetComponent<GhostBehaviour>(out GhostBehaviour ghost))
        {
            if (canAttack)
            {
                hasEatenGhost = true;

                AddReward(0.3f/(float)numGhostsToActivate);
                ghost.Die();
            }
            else
            {
                AddReward(-0.3f);
                EndEpisode();
            }

        }

    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.gameObject.CompareTag("pallets"))
        {
            float timeDiff = Time.time - lastPalletEatenTime;

            AddReward(0.3f/(float)numPalletsToActivate);

            lastPalletEatenTime = Time.time;
            other.gameObject.SetActive(false);

            pelletsEaten++;

            // Check if all pellets are eaten
            if (pelletsEaten >= numPalletsToActivate)
            {
                AddReward(0.3f);  // Give positive reward
                Debug.Log("Level Passed, Advancing Lesson.");               
                lessonManager.AddSuccedLesson();                      
                EndEpisode();     // Restart the episode
            }
        }
        
    }

    private void OnTriggerStay(Collider other)
    {
        if (other.gameObject.CompareTag("EnemyZone"))
        {
            AddReward(-0.05f);  // Continuously give negative reward
        }
    }

    private void ActivatePallets(float nPalletsToActivate)
    {
        // Ensure you're shuffling or selecting pallets randomly if that's desired
        List<PalletsBehaviour> shuffledPallets = palletObjects
            .Select(p => p.GetComponent<PalletsBehaviour>())
            .Where(p => p != null)
            .OrderBy(p => UnityEngine.Random.value)
            .ToList();

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
        Transform roomTransform = this.transform.parent;

        // Draw Pallets
        DrawWaypointsGizmos(waypoints, Color.red, roomTransform);
    }

    void DrawWaypointsGizmos(List<Vector3> waypoints, Color color, Transform roomTransform)
    {
        if (waypoints.Count > 0)
        {
            Gizmos.color = color;
            for (int i = 0; i < waypoints.Count - 1; i++)
            {
                Vector3 worldPoint1 = roomTransform.TransformPoint(waypoints[i]);
                Vector3 worldPoint2 = roomTransform.TransformPoint(waypoints[i + 1]);
                Gizmos.DrawLine(worldPoint1, worldPoint2);
            }
        }
    }



}

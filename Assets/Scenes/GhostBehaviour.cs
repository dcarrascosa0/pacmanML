using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.AI;

public class GhostBehaviour : MonoBehaviour
{
    public MLAgent pacman;
    public Transform blinkyTransform; // Only for Inky
    public Vector3[] scatterWaypoints;
    public float frightenedTime = 10f; // Time ghosts remain frightened

    private NavMeshAgent navMeshAgent;
    private GhostMode currentMode = GhostMode.Scatter;

    private float modeTime = 0f;
    private float scatterTime = 10f; // Time ghosts spend scattering
    private float chaseTime = 20f;  // Time ghosts spend chasing

    private bool hasReachedDestination = false;

    public Material originalMaterial;

    private Vector3 startPosition;

    private Vector3 relativeCenter;




    public enum GhostMode { Chase, Scatter, Frightened }

    private void Awake()
    {

        originalMaterial = Instantiate(GetComponent<MeshRenderer>().material);
        // Find pacman, assuming it's located in "room->Agent"
        pacman = transform.parent.parent.Find("Agent").GetComponent<MLAgent>();

        // Find Blinky, assuming it's a sibling to this GameObject under "Ghosts"
        blinkyTransform = transform.parent.Find("Blinky").transform;

        relativeCenter = transform.parent.parent.position;

    }

    void Start()
    {
        navMeshAgent = GetComponent<NavMeshAgent>();
        SwitchMode(GhostMode.Scatter);
        modeTime = Time.time;
        AddWaypoint(relativeCenter + new Vector3(-113.0f, 10.5f, -82.0f));
        AddWaypoint(relativeCenter + new Vector3(-19.0f, 10.5f, -82.0f));
        AddWaypoint(relativeCenter + new Vector3(-19.0f, 10.5f, 16.0f));
        AddWaypoint(relativeCenter + new Vector3(-113.0f, 10.5f, 16.7f));

        startPosition = transform.position;  // Store the starting position

    }

    void Update()
    {
        if (navMeshAgent.remainingDistance <= navMeshAgent.stoppingDistance)
        {
            hasReachedDestination = true;
        }

        switch (currentMode)
        {
            case GhostMode.Chase:
                hasReachedDestination = false; // Reset flag
                Chase();
                break;
            case GhostMode.Scatter:
                if (hasReachedDestination)
                {
                    Scatter();
                    hasReachedDestination = false; // Reset flag
                }
                break;
            case GhostMode.Frightened:
                hasReachedDestination = false; // Reset flag
                Frightened();
                break;
        }

        // Mode transitions
        if (currentMode != GhostMode.Frightened)
        {
            if (currentMode == GhostMode.Scatter && Time.time - modeTime > scatterTime)
            {
                SwitchMode(GhostMode.Chase);
                modeTime = Time.time;
            }
            else if (currentMode == GhostMode.Chase && Time.time - modeTime > chaseTime)
            {
                SwitchMode(GhostMode.Scatter);
                modeTime = Time.time;
            }
        }
    }

    public void SwitchMode(GhostMode newMode)
    {
        CancelInvoke("EndFrightenedMode"); // Cancel any previous calls to end Frightened mode
        currentMode = newMode;
        MeshRenderer renderer = GetComponent<MeshRenderer>();

        if (renderer && renderer.material)
        {
            if (newMode == GhostMode.Frightened)
            {
                renderer.material.color = Color.blue;  // Set to blue
                Invoke("EndFrightenedMode", frightenedTime);
            }
            else
            {
                renderer.material = originalMaterial; // Reset to original material
                modeTime = Time.time;
            }
        }
    }

    private void EndFrightenedMode()
    {
    
        SwitchMode(GhostMode.Chase); // Or back to previous mode
        navMeshAgent.speed = 15;
    }



    private void Chase()
    {
        Vector3 target = Vector3.zero;
        if (gameObject.name == "Blinky")
        {
            target = pacman.transform.position;
        }
        else if (gameObject.name == "Pinky")
        {
            target = pacman.transform.position + new Vector3(pacman.directionX, pacman.transform.position.y, pacman.directionZ) * 5f;
        }
        else if (gameObject.name == "Inky")
        {
            Vector3 blinkyToPacman = pacman.transform.position - blinkyTransform.position;
            target = pacman.transform.position + blinkyToPacman * 0.5f;
        }
        else if (gameObject.name == "Clyde")
        {
            float distanceToPacman = Vector3.Distance(transform.position, pacman.transform.position);
            if (distanceToPacman > 8)
                target = pacman.transform.position;
            else
            {
                Scatter();
                return;
            }
        }

        // Clamp the new position within the defined bounds
        target = ClampInRelativeSpace(target);



        navMeshAgent.SetDestination(target);
    }


    private void Scatter()
    {
        if (hasReachedDestination)
        {
            Vector3 scatterTarget = scatterWaypoints[Random.Range(0, scatterWaypoints.Length)];
            navMeshAgent.SetDestination(scatterTarget);
        }
    }

    private void Frightened()
    {
        Vector3 farthestWaypoint = Vector3.zero;
        float maxDistance = float.MinValue;

        foreach (Vector3 waypoint in scatterWaypoints)
        {
            float distance = Vector3.Distance(waypoint, pacman.transform.position);
            float distGhostToWaypoint = Vector3.Distance(transform.position, waypoint);
            float distPacmanToWaypoint = Vector3.Distance(pacman.transform.position, waypoint);

            if (distance > maxDistance && distPacmanToWaypoint > distGhostToWaypoint)
            {
                maxDistance = distance;
                farthestWaypoint = waypoint;
            }
        }

        if (pacman.IsMoving())
        {
            // Calculate opposite direction
            Vector3 oppositeDirection = (transform.position - pacman.transform.position).normalized;
            Vector3 oppositeTarget = transform.position + oppositeDirection * 5f; // You can adjust the multiplier

            oppositeTarget = ClampInRelativeSpace(oppositeTarget);
            navMeshAgent.SetDestination(oppositeTarget);
        }
        else
        {
            navMeshAgent.SetDestination(farthestWaypoint);
        }
    }




    public void AddWaypoint(Vector3 newWaypoint)
    {
        int currentLength = scatterWaypoints.Length;
        System.Array.Resize(ref scatterWaypoints, currentLength + 1);
        scatterWaypoints[currentLength] = newWaypoint;
    }

    public void EnterFrightenedMode()
    {
        SwitchMode(GhostMode.Frightened);
        navMeshAgent.speed = 5;
    }

    public void Die()
    {
        Reset();

    }

    public void Reset()
    {
        // Reset position
        navMeshAgent.Warp(startPosition);
        navMeshAgent.isStopped = false;

        // Reset ghost mode and timer
        SwitchMode(GhostMode.Scatter);
        modeTime = Time.time;

        // Reset materials and flags
        MeshRenderer renderer = GetComponent<MeshRenderer>();
        if (renderer && renderer.material)
        {
            renderer.material = originalMaterial;
        }
        hasReachedDestination = false;

        // Reactivate the gameObject in case it was deactivated
        gameObject.SetActive(true);
    }

    private Vector3 ClampInRelativeSpace(Vector3 target)
    {
        float xMin = relativeCenter.x - 113.0f;
        float xMax = relativeCenter.x - 19.0f;
        float zMin = relativeCenter.z - 82.0f;
        float zMax = relativeCenter.z + 16.7f;

        target = new Vector3(
            Mathf.Clamp(target.x, xMin, xMax),
            10.5f,
            Mathf.Clamp(target.z, zMin, zMax)
        );

        return target;
    }






}

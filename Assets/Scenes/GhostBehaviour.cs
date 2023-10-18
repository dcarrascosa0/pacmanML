using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.AI;

public class GhostBehaviour : MonoBehaviour
{
    public MLAgent pacman;
    public Transform blinkyTransform; // Only for Inky
    public Vector3[] scatterWaypoints;
    public float frightenedTime = 10f; // Time ghosts remain frightened

    private NavMeshAgent navMeshAgent;
    public GhostMode currentMode = GhostMode.Scatter;

    private float modeTime = 0f;
    private float scatterTime = 10f; // Time ghosts spend scattering
    private float chaseTime = 20f;  // Time ghosts spend chasing

    private bool hasReachedDestination = false;

    public Material originalMaterial;

    private Vector3 startPosition;

    private Vector3 relativeCenter;
    public Vector3 scatterTarget;

    private Bounds bounds;



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
        Physics.IgnoreLayerCollision(7, 7, true);
        navMeshAgent = GetComponent<NavMeshAgent>();
        SwitchMode(GhostMode.Scatter);
        modeTime = Time.time;
        AddWaypoint(relativeCenter + new Vector3(-113.0f, 10.5f, -82.0f));
        AddWaypoint(relativeCenter + new Vector3(-19.0f, 10.5f, -82.0f));
        AddWaypoint(relativeCenter + new Vector3(-19.0f, 10.5f, 16.0f));
        AddWaypoint(relativeCenter + new Vector3(-113.0f, 10.5f, 16.7f));

        startPosition = transform.position;  // Store the starting position
        CalculateBounds();

    }

    private void CalculateBounds()
    {
        float xMin = float.MaxValue;
        float xMax = float.MinValue;
        float zMin = float.MaxValue;
        float zMax = float.MinValue;

        foreach (Vector3 point in scatterWaypoints)
        {
            if (point.x < xMin) xMin = point.x;
            if (point.x > xMax) xMax = point.x;
            if (point.z < zMin) zMin = point.z;
            if (point.z > zMax) zMax = point.z;
        }

        bounds = new Bounds(new Vector3((xMax + xMin) / 2, 10.5f, (zMax + zMin) / 2), new Vector3(xMax - xMin, 0, zMax - zMin));
    }

    void Update()
    {
        // Check for NavMeshAgent reaching its destination
        if (navMeshAgent.remainingDistance < 2.0f && !navMeshAgent.pathPending)
        {
            hasReachedDestination = true;
        }
        else
        {
            hasReachedDestination = false;
        }

        switch (currentMode)
        {
            case GhostMode.Chase:
                Chase();
                break;
            case GhostMode.Scatter:
                if (hasReachedDestination)
                {
                    Scatter();
                }
                break;
            case GhostMode.Frightened:
                if (hasReachedDestination)
                {
                    Frightened();
                }
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
        currentMode = newMode;     
    }

    public void EndFrightenedMode()
    {
        MeshRenderer renderer = GetComponent<MeshRenderer>();
        renderer.material = originalMaterial; // Reset to original material
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
        
        scatterTarget = scatterWaypoints[Random.Range(0, scatterWaypoints.Length)];
        navMeshAgent.SetDestination(scatterTarget);
        
    }

    private void Frightened()
    {
        
        scatterTarget = GetRandomPointWithinBounds();
        navMeshAgent.SetDestination(scatterTarget);
        
    }

    private Vector3 GetRandomPointWithinBounds()
    {
        float x = Random.Range(bounds.min.x, bounds.max.x);
        float z = Random.Range(bounds.min.z, bounds.max.z);

        return new Vector3(x, 10.5f, z);
    }




    public void AddWaypoint(Vector3 newWaypoint)
    {
        int currentLength = scatterWaypoints.Length;
        System.Array.Resize(ref scatterWaypoints, currentLength + 1);
        scatterWaypoints[currentLength] = newWaypoint;
    }

    public void EnterFrightenedMode()
    {
        MeshRenderer renderer = GetComponent<MeshRenderer>();
        SwitchMode(GhostMode.Frightened);
        navMeshAgent.speed = 5;
        renderer.material.color = Color.blue;  // Set to blue
    }

    public void Die()
    {
        if (navMeshAgent.isOnNavMesh)
        {
            // Reset position
            navMeshAgent.Warp(startPosition);
            navMeshAgent.isStopped = false;
        }
    }

    public void Reset()
    {
        // Reactivate the gameObject in case it was deactivated
        gameObject.SetActive(true);

        if (navMeshAgent.isOnNavMesh)
        {
            // Reset position
            navMeshAgent.Warp(startPosition);
            navMeshAgent.isStopped = false;
        }

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
        Scatter();
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
    public Vector3 GetDirection()
    {
        return navMeshAgent.velocity.normalized;
    }






}

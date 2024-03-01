using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RCSpawnTarget : MonoBehaviour
{

    public Transform referenceCube;
    public float maxRadius = 4.0f;
    public float minRadius = 2.0f;
    // private Vector3 orgin = Vector3.zero;

    void Awake(){
        TargetSpawn();
    }

    public Vector3 GenerateRandomPoint(Vector3 originPosition, float min, float max)
    {
        originPosition = referenceCube.position;
        // Generate a random angle around the y-axis
        float angle = Random.Range(0f, Mathf.PI * 2f);
        
        // Generate a random distance within the specified range
        float distance = Random.Range(min, max);
        
        // Calculate the x and z coordinates based on the angle and distance
        float x = originPosition.x + Mathf.Cos(angle) * distance;
        float z = originPosition.z + Mathf.Sin(angle) * distance;
        
        // Ensure y = 0
        float y = 0f;
        
        // Return the random point
        return new Vector3(x, y, z);
    }

    /// <summary>
    /// Set the position of the target ball to a random position within the specified min/max radius
    /// </summary>
    public void TargetSpawn(){
        // Set the position of the target ball to a random position within the specified min/max radius
        // Vector3 spawnSpot;
        // Vector3 distanceVector;

        // do {
        //     spawnSpot = Random.insideUnitSphere * maxRadius;
        //     spawnSpot.y = 0;
        //     distanceVector = spawnSpot - referenceCube.position;
        // } while (distanceVector.magnitude < minRadius);
        
        // transform.position = referenceCube.position + spawnSpot;

        // Random number between minRadius and maxRadius
        Vector3 spawnSpot = GenerateRandomPoint(referenceCube.position, minRadius, maxRadius);
        transform.position = spawnSpot;
    }
}



using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UtilitiesSpace;
public class ObstacleManager : MonoBehaviour
{
    public GameObject obstaclePrefab;
    public Transform referenceOrigin;
    public LayerMask groundLayer;
    public LayerMask limitsLayer;
    public Transform ground;

    // Start is called before the first frame update
    public void placeObstacles(int numObstacles)
    {
        // Get dimensions of the ground
        float groundWidth;
        float groundHeight;
        (groundWidth, groundHeight) = Utilities.getWidthHeight(ground);
        
        int trials = 0;
        Debug.Log("Placing obstacles");
        for (int i = 0; i < numObstacles; i++)
        {
            bool foundSpot = true;
            Vector3 position;
            do {
                position = Utilities.getRandomGroundPosition(groundWidth, groundHeight, referenceOrigin);
                if (trials > 1000)
                {
                    Debug.Log("Too many trials while placing obstacles");
                    foundSpot = false;
                    break;
                }
                trials++;
            } while (Utilities.isObjectInPosition(position, 3.0f, groundLayer));
            
            if (foundSpot)
                Instantiate(obstaclePrefab, position, Quaternion.Euler(90, 0, 0));

        }
        Debug.Log("Obstacles placed");
    }

    public void clearObstacles()
    {
        GameObject[] obstacles = GameObject.FindGameObjectsWithTag("obstacle");
        foreach (GameObject obstacle in obstacles)
        {
            Destroy(obstacle);
        }
    }

}

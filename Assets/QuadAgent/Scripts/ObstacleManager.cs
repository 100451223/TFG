using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UtilitiesSpace;
public class ObstacleManager : MonoBehaviour
{
    public GameObject obstaclePrefab;
    public Transform obstacleSpawnArea;
    public Transform referenceOrigin;
    public LayerMask groundLayer;
    public LayerMask limitsLayer;
    public Transform ground;
    public Vector3 rotationOffset = new Vector3(0, 0, 0);

    // Start is called before the first frame update
    public void placeObstacles(int numObstacles, Transform parent)
    {
        // Get dimensions of the ground        
        int trials = 0;
        for (int i = 0; i < numObstacles; i++)
        {
            bool foundSpot = true;
            Vector3 position;
            do {
                // position = Utilities.getRandomGroundPosition(groundWidth, groundHeight, referenceOrigin);
                position = Utilities.getRandomPlanePosition(obstacleSpawnArea, referenceOrigin);
                if (trials > 1000)
                {
                    Debug.Log("Too many trials while placing obstacles");
                    foundSpot = false;
                    break;
                }
                trials++;
            } while (Utilities.isObjectInPosition(position, 2.0f, groundLayer));
            

            if (foundSpot)
            {
                GameObject newObstacle = Instantiate(obstaclePrefab, position, Quaternion.Euler(rotationOffset.x, rotationOffset.y, rotationOffset.z));
                newObstacle.transform.parent = parent;
                newObstacle.name = "Obstacle" + i;
            }

        }
    }

    public void clearObstacles(Transform parent)
    {
        foreach (Transform child in parent)
        {
            if (child.gameObject.CompareTag("obstacle"))
            {
                Destroy(child.gameObject);
            }
        }
    }

}

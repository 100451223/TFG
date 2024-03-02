using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace UtilitiesSpace{

    public static class Utilities
    {
        
        /// <summary>
        /// Class to store the information of the joints
        /// </summary>
        /// <param name="origin">
        /// Vector3. The origin position
        /// </param>
        /// <param name="target">
        /// Vector3. The target position
        /// </param>
        /// <param name="x">
        /// Bool. Consider the X axis or not in the distance calculation
        /// </param>
        /// <param name="y">
        /// Bool. Consider the Y axis or not in the distance calculation
        /// </param>
        /// <param name="z">
        /// Bool. Consider the Z axis or not in the distance calculation
        /// </param>
        /// <returns>
        /// Float. The distance between the origin and the target
        /// </returns>
        public static float distanceTo(Vector3 origin, Vector3 target, bool x, bool y, bool z)
        {
            Vector3 scaledOrigin = new Vector3(origin.x * (x ? 1 : 0), origin.y * (y ? 1 : 0), origin.z * (z ? 1 : 0));
            Vector3 scaledTarget = new Vector3(target.x * (x ? 1 : 0), target.y * (y ? 1 : 0), target.z * (z ? 1 : 0));

            return Vector3.Distance(scaledOrigin, scaledTarget);
        }

        /// <summary>
        /// Get the likeness between two vectors based on the dot product
        /// </summary>
        /// <param name="vector1">
        /// First vector
        /// </param>
        /// <param name="vector2">
        /// Second vector
        /// </param>
        /// <returns>
        /// Likeness between the two vectors based on the dot product
        /// </returns>
        public static float vectorLikeness(Vector3 vector1, Vector3 vector2){
            return Vector3.Dot(vector1, vector2);
        }

        /// <summary>
        /// Check if GameObject1 is touching GameObject2
        /// </summary>
        /// <param name="gameObj1">
        /// First GameObject
        /// </param>
        /// <param name="gameObj2">
        /// Second GameObject
        /// </param>
        /// <returns>
        /// True if the GameObjects are touching, false otherwise
        /// </returns>
        public static bool isTouching(Transform gameObj1, Transform gameObj2){
            // Get the target's collider
            
            Collider[] gameObj1Colliders = gameObj1.GetComponentsInChildren<Collider>();
            Collider[] gameObj2Colliders = gameObj2.GetComponentsInChildren<Collider>();
            
            foreach (Collider gameObj1Collider in gameObj1Colliders)
            {
                foreach (Collider gameObj2Collider in gameObj2Colliders)
                {
                    if (gameObj1Collider.bounds.Intersects(gameObj2Collider.bounds))
                    {
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Check if there is an object in the target position
        /// </summary>
        /// <param name="targetPosition">
        /// Position to check
        /// </param>
        /// <param name="checkRadius">
        /// Radius to check
        /// </param>
        /// <param name="groundLayer">
        /// Ground layer (it will and has to be ignored when checking)
        /// </param>
        /// <returns>
        /// True if there is an object in the target position, false otherwise
        /// </returns>
        public static bool isObjectInPosition(Vector3 targetPosition, float checkRadius, LayerMask groundLayer, bool debug = false){

            int layerMask = ~groundLayer;

            if (Physics.CheckSphere(targetPosition, checkRadius, layerMask))
            {
                if (debug) Debug.Log("An object is already in this position");
                return true;
            }
            
            return false;
        }

    }
}

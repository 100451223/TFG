using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class checkElbowGrounded : MonoBehaviour
{
    public Transform groundCheckSphere;
    public float checkRadius;
    public LayerMask groundLayerMask;

    public bool elbowIsGrounded(bool debug = false){
        if (Physics.CheckSphere(groundCheckSphere.position, checkRadius, groundLayerMask))
            {
                if (debug) Debug.Log("Elbow is grounded");
                return true;
            }
        return false;
    }
}

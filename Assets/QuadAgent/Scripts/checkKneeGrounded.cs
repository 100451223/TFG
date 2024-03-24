using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class checkKneeGrounded : MonoBehaviour
{
    public Transform groundCheckSphere;
    public float checkRadius;
    public LayerMask groundLayerMask;

    public bool kneeIsGrounded(bool debug = false){
        if (Physics.CheckSphere(groundCheckSphere.position, checkRadius, groundLayerMask))
            {
                if (debug) Debug.Log("Knee is grounded");
                return true;
            }
        return false;
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class checkPawGrounded : MonoBehaviour
{
    public Transform groundCheckSphere;
    public float checkRadius;
    public LayerMask groundLayerMask;

    public bool pawIsGrounded(bool debug = false){
        if (Physics.CheckSphere(groundCheckSphere.position, checkRadius, groundLayerMask))
            {
                if (debug) Debug.Log("Paw is grounded");
                return true;
            }
        return false;
    }
}

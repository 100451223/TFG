using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class checkGrounded : MonoBehaviour
{
    public Transform groundCheckSphere;
    public float checkRadius;
    public LayerMask groundLayerMask;

    public bool pawIsGrounded(){
        if (Physics.CheckSphere(groundCheckSphere.position, checkRadius, groundLayerMask))
            {
                return true;
            }
        return false;
    }
}

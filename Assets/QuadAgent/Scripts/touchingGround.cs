using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class touchingGround : MonoBehaviour
{
    public float checkRadius = 0.15f;
    public LayerMask groundLayerMask;

    // On gizmos, draw a sphere at the center of mass of the rigidbody
    void OnDrawGizmosSelected()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(GetComponent<Rigidbody>().worldCenterOfMass, checkRadius);
    }

    public bool isTouchingGround(bool debug = false){
        if (Physics.CheckSphere(GetComponent<Rigidbody>().worldCenterOfMass, checkRadius, groundLayerMask))
            {
                if (debug) Debug.Log(transform.name + " is grounded");
                return true;
            }
        return false;
    }
}

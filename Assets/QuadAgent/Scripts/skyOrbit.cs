using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class skyOrbit : MonoBehaviour
{
    public Transform target; 
    public float speed = 10f;

    void OrbitAround()
    {
        // Rotate around the target point at the given speed
        transform.RotateAround(target.position, Vector3.up, speed * Time.deltaTime);
    }

    void Update()
    {
        OrbitAround();
    }
}

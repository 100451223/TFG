using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace JointSpace{
    public class AgentJoint {
        public Transform transform;
        public Rigidbody rigidbody;
        public ConfigurableJoint joint;
        public Vector3 initialPosition;
        public Quaternion initialRotation;
        public Quaternion prevRotation;
        public float lowLimitX;
        public float highLimitX;
        public float limitY;
        public float limitZ;
        public float maxStrength = 20000.0f;
        public float effectiveStrength = 0.0f;
        
        /// <summary>
        ///  Constructor
        /// </summary>
        public AgentJoint (Transform articulation) {

            // Get objects
            transform = articulation;
            rigidbody = articulation.GetComponent<Rigidbody>();
            joint = articulation.GetComponent<ConfigurableJoint>();

            // Get initial setting
            initialPosition = articulation.localPosition;
            initialRotation = articulation.localRotation;

            // Get limits
            lowLimitX = joint.lowAngularXLimit.limit;
            highLimitX = joint.highAngularXLimit.limit;
            limitY = joint.angularYLimit.limit;
            limitZ = joint.angularZLimit.limit;
        }  
    }
}
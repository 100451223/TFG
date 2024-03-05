using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using JointSpace;
using UtilitiesSpace;
using UnityEngine.InputSystem;

public class debugQuadrupedAgent : MonoBehaviour
{

    [Header("Quadruped's Joints")]
    public Transform frontLowerLeg_R;
    public Transform frontLowerLeg_L;
    public Transform frontUpperLeg_R;
    public Transform frontUpperLeg_L;
    public Transform frontWaist_R;
    public Transform frontWaist_L;
    public Transform backLowerLeg_R;
    public Transform backLowerLeg_L;
    public Transform backUpperLeg_R;
    public Transform backUpperLeg_L;
    public Transform backWaist_R;
    public Transform backWaist_L;

    // Dictionary where the key is a string and the value is an AgentJoint
    public Dictionary<Transform, AgentJoint> joints = new Dictionary<Transform, AgentJoint>();
    
    [Header("Speed")]
    public float speed = 10.0f;

    [Header("Target")]
    public Transform target;
    public Transform referenceOrigin;
    
    [Header("Ground")]
    public Transform ground;
    public LayerMask groundLayer;
    private float groundWitdh;
    private float groundHeight;
    
    [Header("Limits")]
    public Transform limits;

    [Header("ML Agents")]
    private float prevDistanceToTarget;

    #region UTILITIES

        public float checkVelocityMatch(){
            Vector3 thisVelocity = GetComponent<Rigidbody>().velocity;
            // Y axis might be causing troubles? Or slowing down convergence?
            // Vector3 agentToTarget = Vector3.Normalize(target.transform.position - transform.position) * speed;
            // agentToTarget.y = 0;
            // Debug.DrawRay(new Vector3(transform.position.x, 0.0f, transform.position.z), agentToTarget, Color.yellow);
            Vector3 targetVelocity = transform.forward * speed;
            return Utilities.vectorLikeness(thisVelocity, targetVelocity);
        }

        /// <summary>
        /// Likeness of the body's normal and the ground's normal right below the body
        /// </summary>
        /// <param name="showGuidelines">
        /// Show the guidelines for the normals in the scene view if true
        /// </param>
        /// <returns>
        /// Likness if there is ground below the body, -2 otherwise
        /// </returns>
        public float checkGroundAlignment(bool debug = true){

            Rigidbody thisRigidbody = GetComponent<Rigidbody>();

            Vector3 agentPosition = thisRigidbody.worldCenterOfMass;
            float raycastLength = 10.0f;
            RaycastHit hit;

            // Draw the normal of the ground below the mainBody
            if (Physics.Raycast(agentPosition, Vector3.down, out hit, raycastLength, groundLayer))
            {
                if(debug){
                    Debug.DrawRay(agentPosition, Vector3.down, Color.green);
                    Debug.DrawRay(agentPosition, transform.up, Color.blue);
                    Debug.DrawRay(hit.point, hit.normal, Color.red);
                }

                float normalsLikeness = Utilities.vectorLikeness(Vector3.Normalize(transform.up), Vector3.Normalize(hit.normal));
                // Debug.Log("Normals Likeness: " + normalsLikeness);
        
                return normalsLikeness;
            }

            return -999.0f; 
        }

        /// <summary>
        /// Check the distance to the ground below the mainBody
        /// </summary>
        /// <returns>
        /// Distance to the ground below the mainBody, -1 if there is no ground below
        /// </returns>
        public float checkGroundDistance(){

            Rigidbody thisRigidbody = GetComponent<Rigidbody>();
            Vector3 agentPosition = thisRigidbody.worldCenterOfMass;
            float raycastLength = 10.0f;
            RaycastHit hit;

            // Return the distance to the ground below the mainBody
            if (Physics.Raycast(agentPosition, Vector3.down, out hit, raycastLength, groundLayer))
            {
                return hit.distance;
            }

            return -1.0f; 
        }        

        /// <summary>
        /// Get a random direction in the XZ plane
        /// </summary>
        public Vector3 getRandomXZDirection()
        {
            Vector3 randomDirection = Random.insideUnitSphere;
            randomDirection.y = 0;
            
            return randomDirection;
        }

        /// <summary>
        /// Get a random position in the ground
        /// </summary>
        /// <param name="limitOffset">
        /// Limit offset to avoid getting a position too close to the limits of the world
        /// </param>
        /// <returns>
        /// Random Vector3 in the ground plane
        /// </returns>
        public Vector3 getRandomGroundPosition(float limitOffset = 0.2f, float heightOffset = 0.0f){

            // Get random position in the ground plane (XZ)
            float randomX = Random.Range(-groundWitdh*(1-limitOffset)/2, groundWitdh*(1-limitOffset)/2);
            float randomZ = Random.Range(-groundHeight*(1-limitOffset)/2, groundHeight*(1-limitOffset)/2);
            
            Vector3 worldPosition = new Vector3(randomX, 0.0f, randomZ);
            // Transform world space position to a position relative to the referenceOrigin
            Vector3 localPosition = referenceOrigin.TransformPoint(worldPosition);
            localPosition.y = heightOffset;

            return localPosition;
        }

        

    #endregion


    #region SETUP - INITIALIZE

        /// <summary>
        /// Add the articulations to the dictionary
        /// </summary>
        public void setUpJoints(){
            joints.Add(frontLowerLeg_R, new AgentJoint(frontLowerLeg_R));
            joints.Add(frontLowerLeg_L, new AgentJoint(frontLowerLeg_L));
            joints.Add(frontUpperLeg_R, new AgentJoint(frontUpperLeg_R));
            joints.Add(frontUpperLeg_L, new AgentJoint(frontUpperLeg_L));
            joints.Add(frontWaist_R, new AgentJoint(frontWaist_R));
            joints.Add(frontWaist_L, new AgentJoint(frontWaist_L));
            joints.Add(backLowerLeg_R, new AgentJoint(backLowerLeg_R));
            joints.Add(backLowerLeg_L, new AgentJoint(backLowerLeg_L));
            joints.Add(backUpperLeg_R, new AgentJoint(backUpperLeg_R));
            joints.Add(backUpperLeg_L, new AgentJoint(backUpperLeg_L));
            joints.Add(backWaist_R, new AgentJoint(backWaist_R));
            joints.Add(backWaist_L, new AgentJoint(backWaist_L));
        
            return;
        }

        /// <summary>
        /// Get the width and height of the ground
        /// </summary>
        public void getGroundWidthHeight(){
            groundWitdh = ground.GetComponent<MeshRenderer>().bounds.size.x;
            groundHeight = ground.GetComponent<MeshRenderer>().bounds.size.z;
            return;
        }

        /// <summary>
        /// Set a random position for the target
        /// </summary>
        public void setRandomTargetPosition(){
            Vector3 newPosition;
            do {
                newPosition = getRandomGroundPosition();
            } while (Utilities.isObjectInPosition(newPosition, 2.0f, groundLayer));
            target.position = newPosition;
            
            // Set random z rotation
            target.rotation = Quaternion.Euler(0.0f, Random.Range(0.0f, 360.0f), 0.0f);

            return;
        }

        /// <summary>
        /// Set a random position for the agent
        /// </summary>
        public void setRandomAgentPosition(){
            Vector3 newPosition;
            do {
                newPosition = getRandomGroundPosition(limitOffset: 0.3f, heightOffset: 1.0f);
            } while (Utilities.isObjectInPosition(newPosition, 2.0f, groundLayer));
            transform.position = newPosition;
            
            // Set random z rotation
            transform.rotation = Quaternion.Euler(0.0f, Random.Range(0.0f, 360.0f), 0.0f);

            return;
        }

        /// <summary>
        /// Reset the velocity of the joints
        // /// </summary>
        public void resetJoints(){
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                joint.Value.transform.localPosition = joint.Value.initialPosition;
                joint.Value.transform.localRotation = joint.Value.initialRotation;
                joint.Value.rigidbody.velocity = Vector3.zero;
                joint.Value.rigidbody.angularVelocity = Vector3.zero;
            }

            return;
        }

        /// <summary>
        /// Set the agent to a new random position, resetting the joints
        /// </summary>
        public void ResetAgent()
        {
            resetJoints();
            setRandomAgentPosition();

            return;
        }

        /// <summary>
        /// Set the target to a new random position
        /// </summary>
        public void ResetTarget()
        {
            setRandomTargetPosition();

            return;   
        }

    #endregion

    #region MOVEMENT

        /// <summary>  
        /// Clamp and map the value of the action to the range [min, max]
        /// </summary>
        public float clampActionValue(float value, float min, float max){

            // value is in range [-1, 1], so we will normalize it to [0, 1]
            float minMaxValue = (value + 1f) * 0.5f;
            // Map the value to the range [min, max]
            float result = Mathf.Lerp(min, max, minMaxValue);
            
            return result;
        }

        public Transform joint;
        [Range (-1f, 1f)]
        public float x;
        [Range (-1f, 1f)]
        public float y;
        [Range (-1f, 1f)]
        public float z;
        public float strength;
        /// <summary>
        /// Move the articulation to the target rotation with the given strength
        /// </summary>
        public void moveJoint(Transform joint, float x, float y, float z, float strength){
            
            if (joint == null) return;

            // Get the articulation
            AgentJoint articulation = joints[joint];

            // STRENGTH
            // Normalize the strength value
            float normalizedStrength = (strength + 1f) * 0.5f;
            // Calculate the effective strength, limited by the max strength of the joint
            float effectiveStrength = articulation.maxStrength * normalizedStrength;

            // Set strength
            JointDrive jd = new JointDrive
            {
                positionSpring = 50000,
                positionDamper = 10000,
                maximumForce = strength
            };
            articulation.joint.slerpDrive = jd;
            articulation.effectiveStrength = effectiveStrength;

            // ROTATION
            // Clamp and map the value of the action to the range [min, max]
            float clampValueX = clampActionValue(x, articulation.lowLimitX, articulation.highLimitX);
            float clampValueY = clampActionValue(y, -articulation.limitY, articulation.limitY);
            float clampValueZ = clampActionValue(z, -articulation.limitZ, articulation.limitZ);

            // Target rotation is expected as a Quaternion
            Quaternion targetRotation = Quaternion.Euler(clampValueX, clampValueY, clampValueZ);
            // Set the target rotation of the articulation
            articulation.joint.targetRotation = targetRotation;
        }

    #endregion

    #region COLLISIONS

        /// <summary>
        /// Check if the right and left legs are colliding. Only crossed legs are considered,
        /// but not the legs of the same side.
        /// </summary>
        /// <returns>
        /// True if any pair of legs are colliding, false otherwise
        /// </returns>
        public bool legsColliding(bool debug=false) {

            bool frontLegsCol = Utilities.isTouching(frontLowerLeg_R, frontLowerLeg_L) || Utilities.isTouching(frontLowerLeg_R, frontUpperLeg_L) || 
                                Utilities.isTouching(frontUpperLeg_R, frontLowerLeg_L) || Utilities.isTouching(frontUpperLeg_R, frontUpperLeg_L);

            bool backLegsCol =  Utilities.isTouching(backLowerLeg_R, backLowerLeg_L) || Utilities.isTouching(backLowerLeg_R, backUpperLeg_L) || 
                                Utilities.isTouching(backUpperLeg_R, backLowerLeg_L) || Utilities.isTouching(backUpperLeg_R, backUpperLeg_L);
            
            if (debug){
                if (frontLegsCol) Debug.Log("Front legs are colliding");
                if (backLegsCol) Debug.Log("Back legs are colliding");
            }

            return frontLegsCol || backLegsCol;
        }

        /// <summary>
        /// Check if the agent has fallen
        /// </summary>
        /// <returns>
        /// True if the agent has fallen, false otherwise
        /// </returns>
        public bool agentFell(float threshold = 0.5f, bool debug=false)
        {
            float angle = checkGroundAlignment();

            // if (debug) Debug.Log("Angle: " + angle);

            if (angle < threshold){
                if (debug) Debug.Log("Agent fell");
                return true;
            }
            return false;
        }

        /// <summary>
        /// Check if the agent is touching the target
        /// </summary>
        /// <returns>
        /// True if the agent is touching the target, false otherwise
        /// </returns>
        public bool touchingTarget(bool debug=false)
        {
            if(Utilities.isTouching(transform, target))
            {
                if (debug) Debug.Log("Agent is touching the target");
                return true;
            }
            return false;
        }

        /// <summary>
        /// Check if the agent is touching the limits
        /// </summary>
        /// <returns>
        /// True if the agent is touching the limits, false otherwise
        /// </returns>
        public bool touchingLimits(bool debug=false){
            if(Utilities.isTouching(transform, limits))
            {
                if (debug) Debug.Log("Agent is touching the limits");
                return true;
            }
            return false;
        }

    #endregion

    #region DEBUG    

        public void drawToTargetGuideline(){
            Vector3 agentForward = Vector3.Normalize(transform.forward);
            agentForward.y = 0;
            Vector3 agentToTarget = Vector3.Normalize(target.transform.position - transform.position);
            agentToTarget.y = 0;
            
            Debug.DrawRay(new Vector3(transform.position.x, 0.0f, transform.position.z), agentForward * 10, Color.blue);
            Debug.DrawRay(new Vector3(transform.position.x, 0.0f, transform.position.z), agentToTarget * 10, Color.red);
        }

        // void OnDrawGizmos(){
        //     Vector3 targetPos = target.position;
        //     // Draw green sphere at target position
        //     Gizmos.color = Color.green;
        //     Gizmos.DrawSphere(targetPos, 0.5f);

        //     Vector3 agentPos = transform.position;
        //     // Draw red sphere at agent position
        //     Gizmos.color = Color.red;
        //     Gizmos.DrawSphere(agentPos, 0.5f);
        // }

    #endregion

    // Start is called before the first frame update
    
    void Start()
    {
        // Set up the joints: save initial position, rotation & others
        setUpJoints();
        // Get the ground width and height of the ground
        getGroundWidthHeight();
        // Set random starting position for the target
        // setRandomTargetPosition();
        // Set random starting position for the agent
        // setRandomAgentPosition();
        // Set the agents distance to the target
        prevDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);
    }


    // Update is called once per frame
    void Update()
    {
        drawToTargetGuideline();
        agentFell(debug: true);
        moveJoint(joint, x, y, z, strength);
        if(Input.GetKeyDown(KeyCode.Space)){
            ResetTarget();
            ResetAgent();
        }
    }

}

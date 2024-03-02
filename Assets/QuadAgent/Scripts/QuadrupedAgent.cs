using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using JointSpace;
using UtilitiesSpace;
using UnityEngine.InputSystem;

public class QuadrupedAgent : Agent
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

        /// <summary>
        /// Move the articulation to the target rotation with the given strength
        /// </summary>
        public void moveJoint(Transform joint, float x, float y, float z, float strength){
            
            if (joint == null) return;

            // Get the articulation
            AgentJoint articulation = joints[joint];

            // STRENGTH
            // Normalize the strength value
            // float normalizedStrength = (strength + 1f) * 0.5f;
            // Calculate the effective strength, limited by the max strength of the joint
            // float effectiveStrength = articulation.maxStrength * normalizedStrength;

            // Set strength
            JointDrive jd = new JointDrive
            {
                positionSpring = 50000,
                positionDamper = 10000,
                maximumForce = strength
            };
            articulation.joint.slerpDrive = jd;
            // articulation.effectiveStrength = effectiveStrength;

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

            if (debug) Debug.Log("Angle: " + angle);

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

    #region MLAGENTS

        public override void Initialize(){
            // Set up the joints: save initial position, rotation & others
            setUpJoints();
            // Get the ground width and height of the ground
            getGroundWidthHeight();
            // Set random starting position for the target
            setRandomTargetPosition();
            // Set random starting position for the agent
            setRandomAgentPosition();
            // Set the agents distance to the target
            
            prevDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);
        }

        public override void OnEpisodeBegin()
        {
            // Reset the joints: initial position and rotation, zero out velocity. Set random position for the agent
            ResetAgent();
            // Set random starting position for the target
            ResetTarget();
        }

        /// <summary>
        /// Add a negative reward if the agent is sliding
        /// </summary>
        void addSlidePunishment(){
            if (backLowerLeg_L.GetComponent<checkGrounded>().pawIsGrounded()){
                // If the foot is touching the ground, linear velocity should be near 0
                // Add a negative reward bigger the further the velocity is from 0
                AddReward(Mathf.Abs((joints[backLowerLeg_L].rigidbody.velocity.x + 
                           joints[backLowerLeg_L].rigidbody.velocity.y + 
                           joints[backLowerLeg_L].rigidbody.velocity.z)/3) * -0.1f);
            }

            if (backLowerLeg_R.GetComponent<checkGrounded>().pawIsGrounded()){
                // If the foot is touching the ground, linear velocity should be near 0
                // Add a negative reward bigger the further the velocity is from 0
                AddReward(Mathf.Abs((joints[backLowerLeg_R].rigidbody.velocity.x + 
                           joints[backLowerLeg_R].rigidbody.velocity.y + 
                           joints[backLowerLeg_R].rigidbody.velocity.z)/3) * -0.1f);
            }

            if (frontLowerLeg_L.GetComponent<checkGrounded>().pawIsGrounded()){
                // If the foot is touching the ground, linear velocity should be near 0
                // Add a negative reward bigger the further the velocity is from 0
                AddReward(Mathf.Abs((joints[frontLowerLeg_L].rigidbody.velocity.x + 
                           joints[frontLowerLeg_L].rigidbody.velocity.y + 
                           joints[frontLowerLeg_L].rigidbody.velocity.z)/3) * -0.1f);
            }

            if (frontLowerLeg_R.GetComponent<checkGrounded>().pawIsGrounded()){
                // If the foot is touching the ground, linear velocity should be near 0
                // Add a negative reward bigger the further the velocity is from 0
                AddReward(Mathf.Abs((joints[frontLowerLeg_R].rigidbody.velocity.x + 
                           joints[frontLowerLeg_R].rigidbody.velocity.y + 
                           joints[frontLowerLeg_R].rigidbody.velocity.z)/3) * -0.1f);
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Position of the target relative to the reference origin of this agent (size 3)
            sensor.AddObservation(target.position);

            // Height of the agent relative to the ground (size 1)
            sensor.AddObservation(checkGroundDistance());

            // Rotation of the agent relative to the reference origin of this agent (size 4)
            Vector3 agentForward = Vector3.Normalize(transform.forward);
            agentForward.y = 0;
            Vector3 agentToTarget = Vector3.Normalize(target.transform.position - transform.position);
            agentToTarget.y = 0;

            sensor.AddObservation(Quaternion.FromToRotation(agentForward, agentToTarget));

            // Rotation, angular velocity of the joint's allowed axis (size 24)
            // The upper and lower legs only rotate on the X axis
            sensor.AddObservation(frontLowerLeg_L.localRotation.x);
            sensor.AddObservation(joints[frontLowerLeg_L].rigidbody.angularVelocity.x);
            sensor.AddObservation(frontUpperLeg_L.localRotation.x);
            sensor.AddObservation(joints[frontUpperLeg_L].rigidbody.angularVelocity.x);
            sensor.AddObservation(frontLowerLeg_R.localRotation.x);
            sensor.AddObservation(joints[frontLowerLeg_R].rigidbody.angularVelocity.x);
            sensor.AddObservation(frontUpperLeg_R.localRotation.x);
            sensor.AddObservation(joints[frontUpperLeg_R].rigidbody.angularVelocity.x);
            sensor.AddObservation(backLowerLeg_R.localRotation.x);
            sensor.AddObservation(joints[backLowerLeg_R].rigidbody.angularVelocity.x);
            sensor.AddObservation(backUpperLeg_R.localRotation.x);
            sensor.AddObservation(joints[backUpperLeg_R].rigidbody.angularVelocity.x);
            sensor.AddObservation(backLowerLeg_L.localRotation.x);
            sensor.AddObservation(joints[backLowerLeg_L].rigidbody.angularVelocity.x);
            sensor.AddObservation(backUpperLeg_L.localRotation.x);
            sensor.AddObservation(joints[backUpperLeg_L].rigidbody.angularVelocity.x);
            // The waist only rotates on the Z axis
            sensor.AddObservation(frontWaist_L.localRotation.z);
            sensor.AddObservation(joints[frontWaist_L].rigidbody.angularVelocity.z);
            sensor.AddObservation(frontWaist_R.localRotation.z);
            sensor.AddObservation(joints[frontWaist_R].rigidbody.angularVelocity.z);
            sensor.AddObservation(backWaist_L.localRotation.z);
            sensor.AddObservation(joints[backWaist_L].rigidbody.angularVelocity.z);
            sensor.AddObservation(backWaist_R.localRotation.z);
            sensor.AddObservation(joints[backWaist_R].rigidbody.angularVelocity.z);


            // isFootGrounded, linear velocity (size 4, 4*3)
            sensor.AddObservation(frontLowerLeg_R.GetComponent<checkGrounded>().pawIsGrounded());
            sensor.AddObservation(joints[frontLowerLeg_R].rigidbody.velocity);
            sensor.AddObservation(frontLowerLeg_L.GetComponent<checkGrounded>().pawIsGrounded());
            sensor.AddObservation(joints[frontLowerLeg_L].rigidbody.velocity);
            sensor.AddObservation(backLowerLeg_R.GetComponent<checkGrounded>().pawIsGrounded());
            sensor.AddObservation(joints[backLowerLeg_R].rigidbody.velocity);
            sensor.AddObservation(backLowerLeg_L.GetComponent<checkGrounded>().pawIsGrounded());
            sensor.AddObservation(joints[backLowerLeg_L].rigidbody.velocity);

            // This Velocity (3)
            Vector3 thisVelocity = GetComponent<Rigidbody>().velocity;
            sensor.AddObservation(thisVelocity);

            sensor.AddObservation(Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true));
        
            // Total observation space size: 3 + 1 + 4 + 24 + 4 + 4*3 + 3 = 49
        
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            int i = 0;
            //           JOINT                      X Axis                   Y Axis                 Z Axis                Strength  
            moveJoint(frontLowerLeg_L, actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(frontLowerLeg_R, actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(frontUpperLeg_L, actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(frontUpperLeg_R, actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(frontWaist_L,    0 ,                                      0,   actionBuffers.ContinuousActions[i++], 1000);
            moveJoint(frontWaist_R,    0 ,                                      0,   actionBuffers.ContinuousActions[i++], 1000);
            moveJoint(backLowerLeg_L,  actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(backLowerLeg_R,  actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(backUpperLeg_L,  actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(backUpperLeg_R,  actionBuffers.ContinuousActions[i++],    0,   0,                                    1000);
            moveJoint(backWaist_L,     0,                                       0,   actionBuffers.ContinuousActions[i++], 1000);
            moveJoint(backWaist_R,     0,                                       0,   actionBuffers.ContinuousActions[i++], 1000);

            // Reward the agent if it is aligning with the ground
            float groundAlignmentLikeness = checkGroundAlignment();
            float groundAlignmentLikenessNormalized = (groundAlignmentLikeness + 1.0f) * 0.5f;
            AddReward(- (1 - groundAlignmentLikenessNormalized));
            if (- (1 - groundAlignmentLikenessNormalized) > 0){
                Debug.Log("fuck ground alignment");
            }
            if (groundAlignmentLikeness == -999){
                Debug.Log("fuck where is the ground");
            }

            // Reward the agent if the velocity of the body is similar to the target velocity
            
            // float velocityLikenessNormalized = (velocityLikeness + 1.0f) * 0.5f;
            float velocityLikeness = checkVelocityMatch();
            float velocityClamped01 = Mathf.Clamp01(velocityLikeness);
            AddReward(- (1 - velocityClamped01));
            if(- (1 - velocityClamped01) > 0){
                Debug.Log("fuck velocity match");
            }

            // Punish the agent for sliding
            addSlidePunishment();

            // Reward reduction of distance to the target
            float currentDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);
            AddReward(Mathf.Clamp(1-(currentDistanceToTarget/prevDistanceToTarget), -1, 1));
            prevDistanceToTarget = currentDistanceToTarget;
        }

        void FixedUpdate()
        {

            // Reward the agent if it is touching the target
            if (touchingTarget()){
                AddReward(10.0f);
                EndEpisode();
            }

            // Punish the agent if it fell 
            if (agentFell()){
                AddReward(-1.0f);
                EndEpisode();
            }

            // Punish the agent if it is touching the limits
            if (touchingLimits()){
                AddReward(-1.0f);
            }

            drawToTargetGuideline();
        }

    #endregion

    // Start is called before the first frame update
    
    // void Start()
    // {
    //     setUpJoints();
    //     getGroundWidthHeight();
    //     setRandomTargetPosition();
    //     setRandomAgentPosition();
    // }


    // Update is called once per frame
    // void Update()
    // {
    //     drawToTargetGuideline();
    //     agentFell();
    //     // moveJoint();
    //     if(Input.GetKeyDown(KeyCode.Space)){
    //         ResetTarget();
    //         ResetAgent();
    //     }
    // }
}

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
    public float targetSpeed = 10.0f;

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
    private float maxDistanceToGround;

    #region UTILITIES

        public float checkVelocityMatch(){
                Vector3 thisVelocity = GetComponent<Rigidbody>().velocity;
                // Y axis might be causing troubles? Or slowing down convergence?
                Vector3 targetVelocity = transform.forward * targetSpeed;
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
        /// Check the distance from the main bodie's center of mass to the ground below
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

        /// <summary>
        /// Get the average velocity of the joints
        /// </summary>
        /// <returns>
        public Vector3 getAvgJointVelocity(){
            Vector3 avgVelocity = Vector3.zero;
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                avgVelocity += joint.Value.rigidbody.velocity;
            }
            avgVelocity += GetComponent<Rigidbody>().velocity;
            return avgVelocity/(joints.Count + 1);
        }


        public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
        {
            //distance between our actual velocity and goal velocity
            var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, targetSpeed);

            //return the value on a declining sigmoid shaped curve that decays from 1 to 0
            //This reward will approach 1 if it matches perfectly and approach zero as it deviates
            return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / targetSpeed, 2), 2);
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
        public void moveJoint(Transform joint, float x, float y, float z, float strength, bool debug = false){
            
            if (joint == null) return;


            // Get the articulation
            AgentJoint articulation = joints[joint];

            // ROTATION
            // Clamp and map the value of the action to the range [min, max]
            float clampValueX = clampActionValue(x, articulation.lowLimitX, articulation.highLimitX);
            float clampValueY = clampActionValue(y, -articulation.limitY, articulation.limitY);
            float clampValueZ = clampActionValue(z, -articulation.limitZ, articulation.limitZ);

            // Target rotation is expected as a Quaternion
            Quaternion targetRotation = Quaternion.Euler(clampValueX, clampValueY, clampValueZ);
            // Set the target rotation of the articulation
            articulation.joint.targetRotation = targetRotation;

            // If its front lower leg R then print the rotation on the X axis and the limits
            if (joint == frontLowerLeg_R && debug){
                Debug.Log("X: " + clampValueX + " | Y: " + clampValueY + " | Z: " + clampValueZ + "; Low Limit X: " + articulation.lowLimitX + " | High Limit X: " + articulation.highLimitX);
            }
            // STRENGTH
            // Normalize the strength value
            float normalizedStrength = (strength + 1f) * 0.5f;
            // Calculate the effective strength, limited by the max strength of the joint
            float effectiveStrength = articulation.maxStrength * normalizedStrength;
            // Set strength
            JointDrive jd = new JointDrive
            {
                positionSpring = 40000,
                positionDamper = 5000,
                maximumForce = effectiveStrength
            };
            articulation.joint.slerpDrive = jd;
            articulation.effectiveStrength = effectiveStrength;
        }

    #endregion

    #region COLLISIONS

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

        void OnDrawGizmos(){
            
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawWireSphere(joint.Value.rigidbody.position, 0.1f);
            }
        }

    #endregion

    #region REWARD FUNCTION

        /// <summary>
        /// Punish the agent if it is touching the limits
        /// </summary>
        /// <returns>
        public void punishTouchingLimits(){
            if (touchingLimits(true)){
                // Debug.Log("Agent is touching the limits (-0.1)");
                AddReward(-0.1f);
            }
        }

        /// <summary>
        /// Falling means resetting the episode with a high negative reward.
        /// Were the reward to be small, the agent could decide that falling is the easiest/quickest way to maximize the reward.
        /// </summary>
        public void punishFalling(){
            if (agentFell()){
                AddReward(-0.1f);
                // Debug.Log("Agent fell");
                // EndEpisode();
            }
        }

        /// <summary>
        /// Reward the agent if it is touching the target
        /// </summary>
        public void rewardTouchingTarget(){
            if (touchingTarget()){
                AddReward(1.0f);
                // Debug.Log("Agent is touching the target (+10)");
                EndEpisode();
            }
        }

        /// <summary>
        /// Reward the agent for decreasing the distance to the ground. Punish it for increasing the distance to the ground
        /// </summary>
        public void rewardDistanceToTarget_easy(float weight = 1.0f){
            
            float currentDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);

            float reward = currentDistanceToTarget < prevDistanceToTarget? 1 * weight : - 1 * weight;
            // Debug.Log("Distance to target reward: " + reward);
            AddReward(reward);
            prevDistanceToTarget = currentDistanceToTarget;
        }

        /// <summary>
        /// Reward the agent for decreasing the minimum distance record to the target. Punish it for increasing the distance to the target
        /// </summary>
        public void rewardDistanceToTarget_hard(float weight = 1.0f){
            
            float currentDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);

            float reward = currentDistanceToTarget < prevDistanceToTarget? 1 * weight : - 1 * weight;
            // Debug.Log("Distance to target reward: " + reward);
            AddReward(reward);
            prevDistanceToTarget = currentDistanceToTarget < prevDistanceToTarget? currentDistanceToTarget : prevDistanceToTarget;
        }

        public void rewardTargetAlignment(float weight = 1.0f){
            Vector3 agentForward = Vector3.Normalize(transform.forward);
            agentForward.y = 0;
            Vector3 agentToTarget = Vector3.Normalize(target.transform.position - transform.position);
            agentToTarget.y = 0;
            float reward = Utilities.vectorLikeness(agentForward, agentToTarget) * weight;
            AddReward(reward);
        }

        public void rewardSmoothMovement(float weight = 1.0f){
            
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                float effectiveStrength = joint.Value.effectiveStrength;
                float maxJointStrength = joint.Value.maxStrength;
                float strengthRatio = effectiveStrength / maxJointStrength;
                float reward = - strengthRatio * weight;

                AddReward(reward);
            }

        }
        


        // /// <summary>
        // /// Punish the agent if it is not aligning with the ground.
        // /// </summary>
        // public void punishGroundAlignment(){
        //     float groundAlignmentLikeness = checkGroundAlignment();
        //     float groundAlignmentLikenessNormalized = (groundAlignmentLikeness + 1.0f) * 0.5f;
        //     float punishment = - (1 - groundAlignmentLikenessNormalized);
        //     AddReward(punishment);
        //     // Debug.Log("Ground Alignment Likeness Punishment: " + punishment);
        // }

        /// <summary>
        /// Punish the agent for walking with the knees
        /// </summary>
        /// <param name="weight">
        /// Weight of the punishment
        /// </param>
        public void punishKneeStep(float weight = 1.0f){
            float frontKneeStepping_R = frontUpperLeg_R.GetComponent<checkElbowGrounded>().elbowIsGrounded()? 1f: 0f;
            float frontKneeStepping_L = frontUpperLeg_L.GetComponent<checkElbowGrounded>().elbowIsGrounded()? 1f: 0f;
            float backKneeStepping_R = backUpperLeg_R.GetComponent<checkElbowGrounded>().elbowIsGrounded()? 1f: 0f;
            float backKneeStepping_L = backUpperLeg_L.GetComponent<checkElbowGrounded>().elbowIsGrounded()? 1f: 0f;

            float punishment = - (frontKneeStepping_R + frontKneeStepping_L + backKneeStepping_R + backKneeStepping_L) * weight;
            AddReward(punishment);
        }

        /// <summary>
        /// Reward the agent for walking correctly, punish him for standing still
        /// </summary>
        /// <param name="weight">
        /// Weight of the punishment
        /// </param>
        public void rewardPawStep(float weight = 0.1f){
            float frontPawStepping_R = frontLowerLeg_R.GetComponent<checkPawGrounded>().pawIsGrounded()? 1f: 0f;
            float frontPawStepping_L = frontLowerLeg_L.GetComponent<checkPawGrounded>().pawIsGrounded()? 1f: 0f;
            float backPawStepping_R = backLowerLeg_R.GetComponent<checkPawGrounded>().pawIsGrounded()? 1f: 0f;
            float backPawStepping_L = backLowerLeg_L.GetComponent<checkPawGrounded>().pawIsGrounded()? 1f: 0f;

            float pawsStepping = (frontPawStepping_R + frontPawStepping_L + backPawStepping_R + backPawStepping_L);
            if (pawsStepping <= 3){
                float reward = 1 * weight;
                AddReward(reward);
            } else {
                float punishment = - 1 * weight;
                AddReward(punishment);
            }
        }

        public void punishBumpyMovement(float lowerLimit = -1, float upperLimit = 1){
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                float effectiveStrength = joint.Value.effectiveStrength;
                float maxJointStrength = joint.Value.maxStrength;
                float strengthRatio = effectiveStrength / maxJointStrength;
                float reward = - Mathf.Lerp(lowerLimit, upperLimit, strengthRatio);

                AddReward(reward);
            }
        }

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
        }

        public override void OnEpisodeBegin()
        {
            // Reset the joints: initial position and rotation, zero out velocity. Set random position for the agent
            ResetAgent();
            // Set random starting position for the target
            ResetTarget();
            // Calculate the initial distance to the target
            prevDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);
            // Calculate the initial distance to the ground
            maxDistanceToGround = checkGroundDistance();
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Position of the target (size 3)
            sensor.AddObservation(target.position);

            // Distance to the ground (size 1)
            sensor.AddObservation(checkGroundDistance());

            // Velocity of the agent (3)
            Vector3 avgJointVelocityVal = getAvgJointVelocity();
            sensor.AddObservation(avgJointVelocityVal);

            // Angular velocity of the agent (3)
            sensor.AddObservation(GetComponent<Rigidbody>().angularVelocity);

            // Rotation vector from the agent's forward to the target's position on the XZ axis (size 4)
            Vector3 agentForward = Vector3.Normalize(transform.forward);
            agentForward.y = 0;
            Vector3 agentToTarget = Vector3.Normalize(target.transform.position - transform.position);
            agentToTarget.y = 0;

            sensor.AddObservation(Quaternion.FromToRotation(agentForward, agentToTarget));

            // Vector forward of the agent (size 3)
            sensor.AddObservation(agentForward); 

            // Vector to the target (size 3)
            sensor.AddObservation(agentToTarget);

            // Distance to the target (size 1)
            sensor.AddObservation(Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true));

            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                // Position, velocity and angular velocity of the rigidbodies of the joints (size 12*(3+3+3) = 108)
                sensor.AddObservation(joint.Value.rigidbody.position);
                sensor.AddObservation(joint.Value.rigidbody.velocity);
                sensor.AddObservation(joint.Value.rigidbody.angularVelocity);

                // Is the joint touching the ground (1 * 12 = 12)
                sensor.AddObservation(Utilities.isTouching(joint.Value.transform, ground) ? 1 : 0);
            }
        
            // Total observation space size: 3 + 1 + 3 + 3 + 4 + 3 + 3 + 1 + 108 + 12 = 141
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            int i = 0;
            //           JOINT                      X Axis                   Y Axis                 Z Axis                 Strength  
            moveJoint(frontLowerLeg_L, actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(frontLowerLeg_R, actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(frontUpperLeg_L, actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(frontUpperLeg_R, actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(frontWaist_L,    0 ,                                      0,   actionBuffers.ContinuousActions[i++], actionBuffers.ContinuousActions[i++]);
            moveJoint(frontWaist_R,    0 ,                                      0,   actionBuffers.ContinuousActions[i++], actionBuffers.ContinuousActions[i++]);
            moveJoint(backLowerLeg_L,  actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(backLowerLeg_R,  actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(backUpperLeg_L,  actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(backUpperLeg_R,  actionBuffers.ContinuousActions[i++],    0,   0,                                    actionBuffers.ContinuousActions[i++]);
            moveJoint(backWaist_L,     0,                                       0,   actionBuffers.ContinuousActions[i++], actionBuffers.ContinuousActions[i++]);
            moveJoint(backWaist_R,     0,                                       0,   actionBuffers.ContinuousActions[i++], actionBuffers.ContinuousActions[i++]);
        

            // rewardDistanceToTarget_easy(weight: 0.1f);
            rewardDistanceToTarget_hard(weight: 0.1f);

            // rewardTargetAlignment();

            // rewardSmoothMovement(0.1f);
            
            // Punish for very strong movements
            // punishBumpyMovement();
            punishKneeStep(weight: 1f);
            rewardPawStep(weight: 0.1f);
            // punishBumpyMovement();


        }

        void FixedUpdate()
        {
            // Reward the agent if it is touching the target
            rewardTouchingTarget();
            // Punish the agent if it fell 
            punishFalling();
            // Punish the agent if it is touching the limits
            punishTouchingLimits();
            
            // Punish the agent for walking with the knees
            // Reward the agent for walking correctly
            // rewardPawStep();

            // Punish the agent for not aligning with the ground
            // Punish the agent for not matching the velocity
            // punishVelocityMatch();
            // Reward/punish the agent for decreasing/increasing the distance to the target
            // Reward/punish the agent for decreasing/increasing the distance to the ground
            // rewardDistanceToGround();
            // Reward the agent for walking correctly
            // rewardWalkingCycle();
            // Punish the agent for touching the ground
            // punishTouchingGround();

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

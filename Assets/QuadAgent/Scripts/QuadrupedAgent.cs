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

    // Paws
    public Transform frontPaw_R;
    public Transform frontPaw_L;
    public Transform backPaw_R;
    public Transform backPaw_L;
    // Knees
    public Transform frontKnee_R;
    public Transform frontKnee_L;
    public Transform backKnee_R;
    public Transform backKnee_L;

    // Dictionary where the key is a string and the value is an AgentJoint
    public Dictionary<Transform, AgentJoint> joints = new Dictionary<Transform, AgentJoint>();
    
    [Header("Speed")]
    public float targetSpeed = 10.0f;

    [Header("Target")]
    public Transform target;
    public Transform referenceOrigin;
    public float targetPawHeight = 0.3f;
    
    [Header("Ground")]
    public Transform ground;
    public LayerMask groundLayer;
    private float groundWitdh;
    private float groundHeight;
    
    [Header("Limits")]
    public Transform limits;

    [Header("ML Agents")]
    private float prevDistanceToTarget;
    private float maxDistanceToGround = -1.0f;

    #region Utilities


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

        // public float checkVelocityMatch(){
        //         Vector3 thisVelocity = GetComponent<Rigidbody>().velocity;
        //         // Y axis might be causing troubles? Or slowing down convergence?
        //         Vector3 targetVelocity = transform.forward * targetSpeed;
        //         return Utilities.vectorLikeness(thisVelocity, targetVelocity);
        // }

        // public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
        // {
        //     //distance between our actual velocity and goal velocity
        //     var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, targetSpeed);

        //     //return the value on a declining sigmoid shaped curve that decays from 1 to 0
        //     //This reward will approach 1 if it matches perfectly and approach zero as it deviates
        //     return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / targetSpeed, 2), 2);
        // }
        

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
        /// Set a random position for the target
        /// </summary>
        public void setRandomTargetPosition(){
            Vector3 newPosition;
            do {
                newPosition = Utilities.getRandomGroundPosition(groundWitdh, groundHeight, referenceOrigin);
            } while (Utilities.isObjectInPosition(newPosition, 2.0f, groundLayer));
            target.position = newPosition;
            
            // Set random z rotation
            Vector3 randomYDirection = Utilities.getRandomDirectionXYZ(x: false, y: true, z: false);
            target.rotation = Quaternion.Euler(randomYDirection);

            return;
        }

        /// <summary>
        /// Set a random position for the agent
        /// </summary>
        public void setRandomAgentPosition(){
            Vector3 newPosition;
            do {
                newPosition = Utilities.getRandomGroundPosition(groundWitdh, groundHeight, referenceOrigin, limitOffset: 0.3f, heightOffset: 1.0f);
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
            
            // Save the previous rotation value
            articulation.prevRotation = articulation.joint.targetRotation;
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

        public void drawKneeToGround(){
            // draw the ray from the knees to the contact point in the ground
            Debug.DrawRay(frontKnee_R.position, Vector3.down * Utilities.checkDistanceToGround(frontKnee_R.position, groundLayer), Color.green);
            Debug.DrawRay(frontKnee_L.position, Vector3.down * Utilities.checkDistanceToGround(frontKnee_L.position, groundLayer), Color.green);
            Debug.DrawRay(backKnee_R.position, Vector3.down * Utilities.checkDistanceToGround(backKnee_R.position, groundLayer), Color.green);
            Debug.DrawRay(backKnee_L.position, Vector3.down * Utilities.checkDistanceToGround(backKnee_L.position, groundLayer), Color.green);
        }

        public void drawPawToGround(){
            // draw the ray from the paws to the contact point in the ground
            Debug.DrawRay(frontPaw_R.position, Vector3.down * Utilities.checkDistanceToGround(frontPaw_R.position, groundLayer), Color.red);
            Debug.DrawRay(frontPaw_L.position, Vector3.down * Utilities.checkDistanceToGround(frontPaw_L.position, groundLayer), Color.red);
            Debug.DrawRay(backPaw_R.position, Vector3.down * Utilities.checkDistanceToGround(backPaw_R.position, groundLayer), Color.red);
            Debug.DrawRay(backPaw_L.position, Vector3.down * Utilities.checkDistanceToGround(backPaw_L.position, groundLayer), Color.red);
        }

        void OnDrawGizmos(){
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(frontPaw_R.position, 0.2f);
            Gizmos.DrawWireSphere(frontPaw_L.position, 0.2f);
            Gizmos.DrawWireSphere(backPaw_R.position, 0.2f);
            Gizmos.DrawWireSphere(backPaw_L.position, 0.2f);

            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(frontKnee_R.position, 0.3f);
            Gizmos.DrawWireSphere(frontKnee_L.position, 0.3f);
            Gizmos.DrawWireSphere(backKnee_R.position, 0.3f);
            Gizmos.DrawWireSphere(backKnee_L.position, 0.3f);
            
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

            float reward = currentDistanceToTarget < prevDistanceToTarget? 1 * weight : -1 * weight;
            // Debug.Log("Distance to target reward: " + reward);
            AddReward(reward);
            prevDistanceToTarget = currentDistanceToTarget;
        }

        /// <summary>
        /// Reward the agent for decreasing the minimum distance record to the target. Punish it for increasing the distance to the target
        /// </summary>
        public void rewardDistanceToTarget_hard(float weight = 1.0f){
            
            float currentDistanceToTarget = Utilities.distanceTo(transform.position, target.position, x: true, y: false, z: true);

            float reward = currentDistanceToTarget < prevDistanceToTarget? 1 * weight : -1 * weight;
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

        public void rewardGroundAlignment(float weight = 1.0f){
            float likeness = checkGroundAlignment();
            
            // Doesn't affect if there is no ground below????
            if (likeness < -1f){
                Debug.Log("No ground below!");
                return;
            }

            if (likeness < 0){
                AddReward(-1 * weight);
            } else {
                float punishment = - (1 - likeness) * weight;
                AddReward(punishment * weight);
            }
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

        public void punishBruteRotations(float weight = 1.0f){
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                Quaternion prevRotation = joint.Value.prevRotation;
                Quaternion targetRotation = joint.Value.joint.targetRotation;
                float angle = Utilities.getQuaternionAngle(prevRotation, targetRotation);
                float normalizedAngle = Utilities.normalizeAngle(angle);

                float punishment = - normalizedAngle * weight;
                AddReward(punishment);
            }
        }

        public void punishBruteRotationsFromInit(float weight = 1.0f){
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                Quaternion prevRotation = joint.Value.initialRotation;
                Quaternion targetRotation = joint.Value.joint.targetRotation;
                float angle = Utilities.getQuaternionAngle(prevRotation, targetRotation);
                float normalizedAngle = Utilities.normalizeAngle(angle);

                float punishment = - normalizedAngle * weight;
                AddReward(punishment);
            }
        }
    

        /// <summary>
        /// Punish the agent for walking with the knees
        /// </summary>
        /// <param name="weight">
        /// Weight of the punishment
        /// </param>
        public void punishKneeStep(float weight = 1.0f){
            float frontKneeStepping_R = frontUpperLeg_R.GetComponent<checkKneeGrounded>().kneeIsGrounded()? 1f: 0f;
            float frontKneeStepping_L = frontUpperLeg_L.GetComponent<checkKneeGrounded>().kneeIsGrounded()? 1f: 0f;
            float backKneeStepping_R = backUpperLeg_R.GetComponent<checkKneeGrounded>().kneeIsGrounded()? 1f: 0f;
            float backKneeStepping_L = backUpperLeg_L.GetComponent<checkKneeGrounded>().kneeIsGrounded()? 1f: 0f;

            float punishment = - (frontKneeStepping_R + frontKneeStepping_L + backKneeStepping_R + backKneeStepping_L) * weight;
            AddReward(punishment);
        }


        /// [SOON TO BE DEPRECATED]
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

        public void punishBumpyMovement(float lowerLimit = -1.0f, float upperLimit = 1.0f, float weight = 1.0f){
            foreach (KeyValuePair<Transform, AgentJoint> joint in joints)
            {
                float effectiveStrength = joint.Value.effectiveStrength;
                float maxJointStrength = joint.Value.maxStrength;
                float strengthRatio = effectiveStrength / maxJointStrength;
                // float reward = - Mathf.Lerp(lowerLimit, upperLimit, strengthRatio) * weight;
                float punishment = - strengthRatio * weight;

                AddReward(punishment);
            }
        }        

        // public void punishPawKneeRelativeDistance(float kneeDistancePerc = 0.75f, float weight = 1.0f){
        //     // Front right
        //     float frontPawToGround_R = Utilities.checkDistanceToGround(frontPaw_R.position, groundLayer);
        //     float frontKneeToGround_R = Utilities.checkDistanceToGround(frontKnee_R.position, groundLayer);
        //     float punishmentFrontPawKnee_R = - frontPawToGround_R/frontKneeToGround_R;

        //     // Front left
        //     float frontPawToGround_L = Utilities.checkDistanceToGround(frontPaw_L.position, groundLayer);
        //     float frontKneeToGround_L = Utilities.checkDistanceToGround(frontKnee_L.position, groundLayer);
        //     float punishmentFrontPawKnee_L = - frontPawToGround_L/frontKneeToGround_L;

        //     // Back right
        //     float backPawToGround_R = Utilities.checkDistanceToGround(backPaw_R.position, groundLayer);
        //     float backKneeToGround_R = Utilities.checkDistanceToGround(backKnee_R.position, groundLayer);
        //     float punishmentBackPawKnee_R = - backPawToGround_R/backKneeToGround_R;

        //     // Back left
        //     float backPawToGround_L = Utilities.checkDistanceToGround(backPaw_L.position, groundLayer);
        //     float backKneeToGround_L = Utilities.checkDistanceToGround(backKnee_L.position, groundLayer);
        //     float punishmentBackPawKnee_L = - backPawToGround_L/backKneeToGround_L;

        //     float punishment = (punishmentFrontPawKnee_R + punishmentFrontPawKnee_L + punishmentBackPawKnee_R + punishmentBackPawKnee_L) * weight;
        //     AddReward(punishment);
        // }

        private float calcPawHeightReward(Transform paw){
            float pawToGround = Utilities.checkDistanceToGround(paw.position, groundLayer);
            float rewardPawHeight = pawToGround/targetPawHeight;
            if (rewardPawHeight > 1.0f){
                return (1-rewardPawHeight);
            }
            return rewardPawHeight;
        }

        public void rewardWalkGait(float tooManyContactsWeight = 1.0f, float crossedContactsWeight = 1.0f, float targetPawHeightWeight = 1.0f){
            
            // Check which legs are touching the gorund 
            bool frontLegRTouchingGround = Utilities.isTouching(frontLowerLeg_R, ground);
            bool frontLegLTouchingGround = Utilities.isTouching(frontLowerLeg_L, ground);
            bool backLegRTouchingGround = Utilities.isTouching(backLowerLeg_R, ground);
            bool backLegLTouchingGround = Utilities.isTouching(backLowerLeg_L, ground);

            // Punish more than 3 or less than 2 legs touching the ground
            float touchingGroundAmount = (frontLegRTouchingGround ? 1 : 0) + (frontLegLTouchingGround ? 1 : 0) + (backLegRTouchingGround ? 1 : 0) + (backLegLTouchingGround ? 1 : 0);
            if (touchingGroundAmount > 3 || touchingGroundAmount < 2){
                AddReward(-1.0f * tooManyContactsWeight);
            }

            // Reward the agent for walking with the correct gait
            // if (touchingGroundAmount == 2)
            // {
            //     if(frontLegRTouchingGround && backLegLTouchingGround){
            //         AddReward(1.0f * crossedContactsWeight);
            //     } else if (frontLegLTouchingGround && backLegRTouchingGround){
            //         AddReward(1.0f  * crossedContactsWeight);
            //     }
            // }

            // Reward the agent for rising the legs up to the target height
            // if (!frontLegRTouchingGround) AddReward(calcPawHeightReward(frontPaw_R) * targetPawHeightWeight);
            // if (!frontLegLTouchingGround) AddReward(calcPawHeightReward(frontPaw_L) * targetPawHeightWeight);
            // if (!backLegRTouchingGround) AddReward(calcPawHeightReward(backPaw_R) * targetPawHeightWeight);
            // if (!backLegLTouchingGround) AddReward(calcPawHeightReward(backPaw_L) * targetPawHeightWeight);


        }

        public void punishTouchingGround(float weight = 1.0f){
            float reward = 0.0f;
            // Upper leg R
            bool touchingGround_frontUpperLeg_R = Utilities.isTouching(frontUpperLeg_R, ground);
            reward += touchingGround_frontUpperLeg_R ? - 1 : 0;
            // Upper leg L
            bool touchingGround_frontUpperLeg_L = Utilities.isTouching(frontUpperLeg_L, ground);
            reward += touchingGround_frontUpperLeg_L ? - 1 : 0;
            // Waist R
            bool touchingGround_frontWaist_R = Utilities.isTouching(frontWaist_R, ground);
            reward += touchingGround_frontWaist_R ? - 1 : 0;
            // Waist L
            bool touchingGround_frontWaist_L = Utilities.isTouching(frontWaist_L, ground);
            reward += touchingGround_frontWaist_L ? - 1 : 0;
            // Upper leg R
            bool touchingGround_backUpperLeg_R = Utilities.isTouching(backUpperLeg_R, ground);
            reward += touchingGround_backUpperLeg_R ? - 1 : 0;
            // Upper leg L
            bool touchingGround_backUpperLeg_L = Utilities.isTouching(backUpperLeg_L, ground);
            reward += touchingGround_backUpperLeg_L ? - 1 : 0;
            // Back Waist R
            bool touchingGround_backWaist_R = Utilities.isTouching(backWaist_R, ground);
            reward += touchingGround_backWaist_R ? - 1 : 0;
            // Waist L
            bool touchingGround_backWaist_L = Utilities.isTouching(backWaist_L, ground);
            reward += touchingGround_backWaist_L ? - 1 : 0;
            // this 
            // float touchingGround_this = Utilities.isTouching(transform, ground);
            // reward += touchingGround_this ? - 1 : 0;

            AddReward(reward * weight);
        }

        public void rewardDistanceToGround(float weight = 1.0f){
            float distanceToGround = Utilities.checkDistanceToGround(transform.position, groundLayer);
            // Debug.Log("currentDTG/maxDTG: " + distanceToGround/maxDistanceToGround);
            float distanceToGroundRatio = distanceToGround/maxDistanceToGround;
            
            if (distanceToGroundRatio > 1.0f)
            {
                float punishment = (1-distanceToGroundRatio) * weight;
                AddReward(punishment);
            }

            float reward = distanceToGroundRatio * weight;
            AddReward(reward);
        }

        public float mean(float[] values){
            float sum = 0.0f;
            foreach (float value in values){
                sum += value;
            }
            return sum/values.Length;
        }

        // Std
        public float std(float[] values){
            
            float meanValue = mean(values);

            float sumOfSquares = 0.0f;
            foreach (float value in values)
            {
                sumOfSquares += Mathf.Pow(value - meanValue, 2);
            }

            return Mathf.Sqrt(sumOfSquares/values.Length);
        }

        public void punishUnevenWaistHeights(float weight = 1.0f){
            float[] waistHeights = new float[4];
            waistHeights[0] = frontWaist_R.position.y;
            waistHeights[1] = frontWaist_L.position.y;
            waistHeights[2] = backWaist_R.position.y;
            waistHeights[3] = backWaist_L.position.y;

            float stdWaistHeights = std(waistHeights);
            float punishment = stdWaistHeights * weight;
            // Debug.Log("Punishment: " + punishment);
            AddReward(-punishment);
        }

    #endregion

    #region MLAGENTS

        public override void Initialize(){
            // Set up the joints: save initial position, rotation & others
            setUpJoints();
            // Get the ground width and height of the ground
            (groundWitdh, groundHeight) = Utilities.getWidthHeight(ground);
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
            if(maxDistanceToGround < 0){
                Vector3 agentPosition = GetComponent<Rigidbody>().worldCenterOfMass;
                maxDistanceToGround = Utilities.checkDistanceToGround(agentPosition, groundLayer);
            }
        }

        public override void CollectObservations(VectorSensor sensor)
        {
            // Position of the target (size 3)
            sensor.AddObservation(target.position);

            // Distance to the ground (size 1)
            Vector3 agentPosition = GetComponent<Rigidbody>().worldCenterOfMass;
            sensor.AddObservation(Utilities.checkDistanceToGround(agentPosition, groundLayer));

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

                // Angular difference between the rotation taken in the last step and the one before that (size 1 * 12 = 12)
                float angle = Utilities.getQuaternionAngle(joint.Value.initialRotation, joint.Value.joint.targetRotation);
                float normalizedAngle = Utilities.normalizeAngle(angle);
                sensor.AddObservation(normalizedAngle);

                // Effective strength in the last step (size 1 * 12 = 12)
                sensor.AddObservation(joint.Value.effectiveStrength);

                // Is the joint touching the ground (1 * 12 = 12)
                sensor.AddObservation(Utilities.isTouching(joint.Value.transform, ground) ? 1 : 0);
            }
        
            // Total observation space size: 3 + 1 + 3 + 3 + 4 + 3 + 3 + 1 + 108 + 12 +12 + 12= 165
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
        

            // STAGE 1
                
                // PHASE 1
                    // rewardDistanceToTarget_easy(weight: 0.5f);
                // // PHASE 2
                    rewardDistanceToTarget_hard(weight: 0.5f);
                    rewardTargetAlignment(weight: 0.5f);

            // STAGE 2
            punishKneeStep(weight: 1f); // max -4
            rewardWalkGait(tooManyContactsWeight: 1f, crossedContactsWeight: 0.25f, targetPawHeightWeight: 0.2f);
            rewardGroundAlignment(weight: 1f);
            punishUnevenWaistHeights(weight: 0.25f);
            rewardDistanceToGround(weight: 0.1f);

        }


        void FixedUpdate()
        {
            // Reward the agent if it is touching the target
            rewardTouchingTarget();
            // Punish the agent if it fell 
            punishFalling();
            // Punish the agent if it is touching the limits
            punishTouchingLimits();

            drawToTargetGuideline();
            drawKneeToGround();
            drawPawToGround();
        }

    #endregion

    // // Start is called before the first frame update
    
    // void Start()
    // {
    //     setUpJoints();
    //     getGroundWidthHeight();
    //     setRandomTargetPosition();
    //     setRandomAgentPosition();
    // }


    // // Update is called once per frame
    // void Update()
    // {
    //     drawToTargetGuideline();
    //     agentFell();
    //     drawKneeToGround();
    //     drawPawToGround();
    //     // moveJoint();
    //     if(Input.GetKeyDown(KeyCode.Space)){
    //         ResetTarget();
    //         ResetAgent();
    //     }
    // }
}

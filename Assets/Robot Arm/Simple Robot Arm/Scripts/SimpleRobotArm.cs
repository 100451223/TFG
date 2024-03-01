using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

public class Articulation {
    public Transform transform;
    public Rigidbody rigidbody;
    public ConfigurableJoint joint;
    public Vector3 initialPosition;
    public Quaternion initialRotation;
    public float lowLimitX;
    public float highLimitX;
    public float limitY;
    public float limitZ;
    public float maxStrength = 10000.0f;
    public float currentForce = 0.0f;

    /// <summary>
    ///  Constructor
    /// </summary>
    public Articulation(Transform articulation) {

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

public class SimpleRobotArm : Agent
{

    [Header("Arm Joints")]
    public Transform armBase;
    public Transform arm0;
    public Transform arm1;
    public Transform gripper;
    // Get gripper's colliders
    public Collider[] gripperColliders;
    // Dictionary where the key is a string and the value is an Articulation
    public Dictionary<string, Articulation> articulations = new Dictionary<string, Articulation>();

    [Header("Target")]
    public Transform target;

    [Header("ML")]
    public float prevLikenessBaseVector = 0.0f;
    public float prevLikenessArm1Vector = 0.0f;
    // public Transform referenceCube;

    #region SETUP - INITIALIZE

    /// <summary>
    /// Set the arm to a random rotation
    /// </summary>
    public void SetUpRandomStartingDirection()
    {
        // Set the arm to a random rotation
        Vector3 startingDir = Random.insideUnitSphere;
        startingDir.y = 0;
        transform.forward = startingDir;
    }

    /// <summary>
    /// Reset the arm to its initial position and rotation. 
    /// Zero out the velocity and angular velocity of the arm.
    /// </summary>
    public void ResetArm()
    {
        foreach (string key in articulations.Keys)
        {
            // Reset the position and rotation of the articulation
            articulations[key].transform.localPosition = articulations[key].initialPosition;
            articulations[key].transform.localRotation = articulations[key].initialRotation;
            // Reset the velocity and angular velocity of the articulation
            articulations[key].rigidbody.velocity = Vector3.zero;
            articulations[key].rigidbody.angularVelocity = Vector3.zero;
        }
        SetUpRandomStartingDirection();
    }

    /// <summary>
    /// Reset the target to a random position
    /// </summary>
    public void ResetTarget()
    {
        target.GetComponent<SpawnTarget>().TargetSpawn();
    }

    /// <summary>
    /// Add the articulations to the dictionary
    /// </summary>
    public void SetUpArticulations(){
        articulations.Add("armBase", new Articulation(armBase));
        articulations.Add("arm0", new Articulation(arm0));
        articulations.Add("arm1", new Articulation(arm1));
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
    /// Move the articulation to the target rotation
    /// </summary>

    // public string articulationName;
    // [Range (-1f, 1f)]
    // public float x;
    // [Range (-1f, 1f)]
    // public float y;
    // [Range (-1f, 1f)]
    // public float z;

    // public void moveArticulation(){
    
    public float strength;
    public void moveArticulation(string articulationName, float x, float y, float z){
        if (articulationName == null || articulationName == "")
        {
            return;
        }

        // Get the articulation
        Articulation articulation = articulations[articulationName];

        // Normalize the strength value
        // float normalizedStrength = (strength + 1f) * 0.5f;

        // Clamp and map the value of the action to the range [min, max]
        float clampValueX = clampActionValue(x, articulation.lowLimitX, articulation.highLimitX);
        float clampValueY = clampActionValue(y, -articulation.limitY, articulation.limitY);
        float clampValueZ = clampActionValue(z, -articulation.limitZ, articulation.limitZ);

        // Set strength
        JointDrive jd = new JointDrive
        {
            positionSpring = 50000,
            positionDamper = 10000,
            maximumForce = strength
        };
        articulation.joint.slerpDrive = jd;

        // Target rotation is expected as a Quaternion
        Quaternion targetRotation = Quaternion.Euler(clampValueX, clampValueY, clampValueZ);

        // Set the target rotation of the articulation
        articulation.joint.targetRotation = targetRotation;
        // articulation.currentForce = normalizedStrength * articulations[articulationName].maxStrength;

    }

    #endregion

    #region UTILITIES

    /// <summary>
    /// Get the likeness between two vectors base on the dot product
    /// </summary>
    public float vectorLikeness(Vector3 vector1, Vector3 vector2){
        return Vector3.Dot(vector1, vector2);
    }

    #endregion

    #region DEBUG

    public void DrawGripperLevelGuidelines()
    {
        Vector3 arm1Up = Vector3.Normalize(arm1.transform.up);
        Vector3 arm1ToTarget = Vector3.Normalize(target.transform.position - articulations["arm1"].rigidbody.worldCenterOfMass);

        Debug.DrawRay(articulations["arm1"].rigidbody.worldCenterOfMass, arm1Up * 5, Color.blue);
        Debug.DrawRay(articulations["arm1"].rigidbody.worldCenterOfMass, arm1ToTarget * 5, Color.red);
    }

    public void DrawBaseLevelGuidelines(){

        Vector3 baseForward = Vector3.Normalize(armBase.transform.forward);
        baseForward.y = 0;
        Vector3 toTarget = Vector3.Normalize(target.transform.position - armBase.transform.position);
        toTarget.y = 0;
        
        Debug.DrawRay(armBase.transform.position, baseForward * 5, Color.blue);
        Debug.DrawRay(armBase.transform.position, toTarget * 5, Color.red);
    }


    bool isTouchingTarget(){
        // Get the target's collider
        Collider targetCollider = target.GetComponent<Collider>();
        
        // Check if any of the colliders in the gripper is touching the target
        foreach (Collider gripperCollider in gripperColliders)
        {
            if (gripperCollider.bounds.Intersects(targetCollider.bounds))
            {
                // Debug.Log("Gripper touched the target");
                return true;
            }
        }

        return false;
    }

    #endregion

    #region MLAGENTS

    public override void OnEpisodeBegin()
    {
        // Reset the arm to a random direction
        // ResetArm();
        // Reset the target
        ResetTarget();
    }

    public override void CollectObservations(VectorSensor sensor)
    {

        // Position of the target
        sensor.AddObservation(target.transform.localPosition);

        // Get the normalized direction from the arm base to the target
        Vector3 baseForward = Vector3.Normalize(armBase.transform.forward);
        baseForward.y = 0;
        Vector3 toTarget = Vector3.Normalize(target.transform.position - armBase.transform.position);
        toTarget.y = 0;

        sensor.AddObservation(Quaternion.FromToRotation(baseForward, toTarget));

        // Get the normalized direction from the arm1 to the target
        Vector3 arm1Up = Vector3.Normalize(arm1.transform.up);
        Vector3 arm1ToTarget = Vector3.Normalize(target.transform.position - articulations["arm1"].rigidbody.worldCenterOfMass);

        sensor.AddObservation(Quaternion.FromToRotation(arm1Up, arm1ToTarget));

        // Rotation off each body part's rigidbody
        foreach (string key in articulations.Keys)
        {
            sensor.AddObservation(articulations[key].rigidbody.transform.localRotation);
        }
        
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        moveArticulation("armBase", 0, actionBuffers.ContinuousActions[0], 0);
        moveArticulation("arm0", actionBuffers.ContinuousActions[1], 0, 0);
        moveArticulation("arm1", actionBuffers.ContinuousActions[2], 0, 0);

        Vector3 baseForward = Vector3.Normalize(armBase.transform.forward);
        baseForward.y = 0;
        Vector3 toTarget = Vector3.Normalize(target.transform.position - armBase.transform.position);
        toTarget.y = 0;

        float likenessBaseVector = vectorLikeness(baseForward, toTarget);
        if (likenessBaseVector > prevLikenessBaseVector)
        {
            AddReward(0.1f);
            prevLikenessBaseVector = likenessBaseVector;
        }
        else
        {
            AddReward(-0.1f);
        }

        Vector3 arm1Up = Vector3.Normalize(arm1.transform.up);
        Vector3 arm1ToTarget = Vector3.Normalize(target.transform.position - articulations["arm1"].rigidbody.worldCenterOfMass);

        float likenessArm1Vector = vectorLikeness(arm1Up, arm1ToTarget);
        if (likenessArm1Vector > prevLikenessArm1Vector)
        {
            AddReward(0.1f);
            prevLikenessArm1Vector = likenessArm1Vector;
        }
        else
        {
            AddReward(-0.1f);
        }

        // if (likenessBaseVector > 0.9f && likenessArm1Vector > 0.9f)
        // {
        //     AddReward(1.0f);
        //     EndEpisode();
        // }
        
    }

    void FixedUpdate()
    {
        if(isTouchingTarget()){
            AddReward(1.0f);
            EndEpisode();
        }
    }

    #endregion


    // Start is called before the first frame update
    void Start()
    {
        // Add the articulations to the dictionary
        SetUpArticulations();
        // Set the arm to a random rotation
        SetUpRandomStartingDirection();
        // Get gripper's colliders
        gripperColliders = gripper.GetComponentsInChildren<Collider>();

        Vector3 baseForward = Vector3.Normalize(armBase.transform.forward);
        baseForward.y = 0;
        Vector3 toTarget = Vector3.Normalize(target.transform.position - armBase.transform.position);
        toTarget.y = 0;
        prevLikenessBaseVector = vectorLikeness(baseForward, toTarget);

        Vector3 arm1Up = Vector3.Normalize(arm1.transform.up);
        Vector3 arm1ToTarget = Vector3.Normalize(target.transform.position - articulations["arm1"].rigidbody.worldCenterOfMass);
        prevLikenessArm1Vector = vectorLikeness(arm1Up, arm1ToTarget);
        
    }

    // Update is called once per frame
    void Update()
    {
        DrawGripperLevelGuidelines();
        DrawBaseLevelGuidelines();
        // moveArticulation();
        // isTouchingTarget();

        // if(Input.GetKeyDown(KeyCode.Space)){
        //     ResetArm();
        //     target.GetComponent<SpawnTarget>().TargetSpawn();
        // }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;

using NWH.Common.Input;
using NWH.DWP2.ShipController;

public class CollisionAvoidance : Agent
{
    private int rand = 1;
    public GameObject[] toggleTerrain;

    // Boat items
    public AdvancedShipController activeShip;

    private Vector3 startPosition;
    private Vector3 lastPosition;
    private Quaternion startRotation;

    public float spawnRadiusX;
    public float spawnRadiusZ;

    private Rigidbody rb;

    public bool useRandomBoatRotation;
    public bool useRandomBoatSpawn;


    // Target items
    public Transform target;

    public float targetLocationX;
    public float targetLocationZ;
    //float[] itargetLocationX = new float[5];
    //float[] itargetLocationZ = new float[5];

    private float[] itargetLocationX = {0f, 0f, 80f, 0f, -142f};
    private float[] itargetLocationZ = {0f, 100f, -200f, 230f, -480f};

    public float targetRadiusX;
    public float targetRadiusZ;
    private float[] itargetRadiusX = {20f, 30f, 0f, 40f, 50f};
    private float[] itargetRadiusZ = {20f, 30f, 0f, 40f, 50f};

    private Vector3 targetPosition;

    public bool useRandomTargetSpawn;

    // Misc
    public bool useVectorObs;
    public bool useAISObs;

    private List<float> boatDistances = new List<float>(); // Distance
    private List<float> boatAngles = new List<float>(); // Heading angle
    private List<float> boatSpeeds = new List<float>(); // Speed
    private List<float> boatType = new List<float>(); // Sailboat

    private float reward;
    private float finalDistance;
    
    public override void OnEpisodeBegin()
    {
        Globals.Episode += 1;
        Reset();
    }

    public void ResetTerrain()
    {
        if(rand == 3)
        {
            rand = 1;
        }
        else
        {
            rand = 3;
        }

        foreach (GameObject item in toggleTerrain)
        {
            item.SetActive(false);
        }

        toggleTerrain[rand].SetActive(true);

        targetLocationX = itargetLocationX[rand];
        targetLocationZ = itargetLocationZ[rand];
        targetRadiusX = itargetRadiusX[rand];
        targetRadiusZ = itargetRadiusZ[rand];
    }

    /// <summary>
    /// Provides respawn of goal object and transform of boat
    /// TODO figure out how to implement rudder -> 0 deg
    /// </summary>
    private void Reset()
    {
        ResetTerrain();
        SetReward(0f);
        activeShip.transform.localRotation = Quaternion.identity;

        // Respawn boat
        if (useRandomBoatSpawn)
        {
            float spawnX = Random.Range(startPosition.x - spawnRadiusX, startPosition.x + spawnRadiusX);
            float spawnZ = Random.Range(startPosition.z - spawnRadiusZ, startPosition.z + spawnRadiusZ);
            Vector3 spawnPosition = new Vector3(spawnX, transform.position.y, spawnZ);
            rb.transform.position = spawnPosition;
        }
        else
        {
            rb.transform.position = startPosition;
        }

        if (useRandomBoatRotation)
        {
            float spawnY = Random.Range(0, 360);
            rb.transform.rotation = Quaternion.Euler(0, spawnY, 0);
        }
        else
        {
            rb.transform.rotation = Quaternion.identity;
        }

        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Delete all instances of "Tag:GoalObject"
        GameObject[] goals = GameObject.FindGameObjectsWithTag("GoalObject");
        for (int i = 0; i < goals.Length; ++i)
        {
            Destroy(goals[i]);
        }

        // Respawn target
        if (useRandomTargetSpawn)
        {
            float targetX = Random.Range(targetLocationX - targetRadiusX, targetLocationX + targetRadiusX);
            float targetZ = Random.Range(targetLocationZ - targetRadiusZ, targetLocationZ + targetRadiusZ);
            Vector3 targetPosition = new Vector3(targetX, 0, targetZ);

            target = Instantiate(target, new Vector3(targetX, 0, targetZ), Quaternion.identity);
            target.gameObject.name = "GoalObject";
        }
        else
        {
            target = Instantiate(target, new Vector3(targetLocationX, 0, targetLocationZ), Quaternion.identity);
            target.gameObject.name = "GoalObject";
        }
        finalDistance = Vector3.Distance(transform.position, target.position);
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        activeShip = GetComponent<AdvancedShipController>();

        startPosition = rb.transform.localPosition;
        startRotation = rb.rotation;
        
        Reset();
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        //ActionSegment<float> continuousActions = actionsOut.ContinuousActions;
        //continuousActions[0] = activeShip.input.Steering;
        //continuousActions[1] = activeShip.input.Throttle;

        var discreteActionsOut = actionsOut.DiscreteActions;
        if (Input.GetKey(KeyCode.D))
        {
            discreteActionsOut[0] = 2;
        }
        else if (Input.GetKey(KeyCode.A))
        {
            discreteActionsOut[0] = 0;
        }
        else
        {
            discreteActionsOut[0] = 1;
        }


        if (Input.GetKey(KeyCode.W))
        {
            discreteActionsOut[1] = 2;
        }
        else if (Input.GetKey(KeyCode.S))
        {
            discreteActionsOut[1] = 0;
        }
        else
        {
            discreteActionsOut[1] = 1;
        }


    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        Globals.ScreenText();
        //ActionSegment<float> continuousActions = actions.ContinuousActions;
        //activeShip.input.Steering = continuousActions[0];
        //activeShip.input.Throttle = continuousActions[1];

        ActionSegment<int> discreteActions = actions.DiscreteActions;
        //GetComponent<Transform>().rotation.y = new Vector3(0.0f, 0.0f, 0.0f); // += discreteActions[0] - 1;

        activeShip.input.Steering = discreteActions[0] - 1;
        activeShip.input.Throttle = discreteActions[1] - 1;

        Update();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // TODO Normalize all
        if (useVectorObs)
        {
            // Gives the velocity and angular velocity of the boat to the sensor.
            //sensor.AddObservation(transform.InverseTransformDirection(rb.velocity).magnitude); // Speed
            //sensor.AddObservation(transform.InverseTransformDirection(rb.angularVelocity).y); // Angular Velocity


            // Sensor for the heading angle of the target compared to the current.
            Vector3 heading = (target.transform.position - transform.position);
            float headingAngle = Vector3.SignedAngle(transform.forward, heading, Vector3.up);
            sensor.AddObservation(headingAngle/180); // Heading angle (MAX = 1)

            // Sensor for the rudder angle
            //sensor.AddObservation(activeShip.rudders[0].Angle/180); // Rudder angle (MAX = 1)
        }

        sensor.AddObservation(Vector3.Distance(transform.position, target.position));
        // Goal Sensor
        //Ray goalRay = new Ray(transform.position, target.position - transform.position);

        //Debug.DrawLine(goalRay.origin, target.position, Color.blue);

        if (useAISObs)
        {
            List<Transform> dynamicObjects = new List<Transform>(); // List of other vessels

            GameObject[] interestingObjects = GameObject.FindGameObjectsWithTag("DynamicObject");
            // Debug.Log(interestingObjects.Length);

            foreach (GameObject interestingObject in interestingObjects)
            {
                dynamicObjects.Add(interestingObject.transform);
            }

            if (spawnRadiusX < 0)
            {
            foreach (Transform t in dynamicObjects) // For all boats and sailboats
            {
                // Sensor for the distance of the boat to the current
                //float distance = ((transform.position.x - t.position.x) * (transform.position.x - t.position.x) + (transform.position.z - t.position.z) * (transform.position.z - t.position.z)); // Distance from the boat to the activeShip


                //if (distance < 500)
                //{
                    //sensor.AddObservation(distance);

                    // Sensor for the heading angle of the boat comparent to the current
                    //Vector3 boatAngle = t.position - transform.position;
                    //float boatAngleAngle = Vector3.SignedAngle(transform.forward, boatAngle, Vector3.up);
                    //sensor.AddObservation(boatAngleAngle / 180);

                    // Sensor for the relative heading of the other vessel
                    //Vector3 heading = (t.transform.position - transform.position);
                    //float headingAngle = Vector3.SignedAngle(transform.forward, heading, Vector3.up);
                    //sensor.AddObservation(headingAngle / 180); // Heading angle (MAX = 1)

                    // Sensor for the speed of the other vessel (uses the parent's rigidbody)
                    //sensor.AddObservation(t.InverseTransformDirection(t.GetComponentInParent<Rigidbody>().velocity).magnitude);

                    // Sensor for the type of the other vessel
                    //sensor.AddObservation(t.GetComponent<DynamicObject>().sailboat);

                    //boatDistances.Add(distance);
                    //boatAngles.Add(boatAngleAngle / 180);
                    //boatSpeeds.Add(t.InverseTransformDirection(t.GetComponentInParent<Rigidbody>().velocity).magnitude);

                    //if (t.GetComponent<DynamicObject>().sailboat) 
                    //{
                    //    boatType.Add(1);
                    //}
                    //else 
                    //{
                    //    boatType.Add(0); 
                    //}

                //}
            }

            }
            
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.TryGetComponent<GoalObject>(out GoalObject goalObject))
        {
            Globals.Success += 1;
            AddReward(+100f);
            EndEpisode();
        }
        else if (other.TryGetComponent<StaticObject>(out StaticObject staticObject))
        {
            AddReward(-1f);
            //EndEpisode();
        }
        else if (other.TryGetComponent<DynamicObject>(out DynamicObject dynamicObject))
        {
            AddReward(-50f);
            EndEpisode();
        }
    }

    private void OnCollisionEnter(Collision collision)
    {
        if(collision.gameObject.CompareTag("Terrain"))
        {
            AddReward(-100f);
            float distance = Vector3.Distance(transform.position, target.position);
            AddReward(-distance / finalDistance);
            EndEpisode();
            Reset();
        }
    }

    void Update()
    {
        if(StepCount >= 0.99*MaxStep)
        {
            float distance = Vector3.Distance(transform.position, target.position);
            AddReward(-50*distance / finalDistance);
            EndEpisode();
        }

        AddReward(-50/MaxStep);

        List<Transform> dynamicObjects = new List<Transform>();
        GameObject[] interestingObjects = GameObject.FindGameObjectsWithTag("DynamicObject");

        foreach (GameObject interestingObject in interestingObjects)
        {
            dynamicObjects.Add(interestingObject.transform);
        }

        foreach (Transform t in dynamicObjects)
        {
            // Debug.Log("GivingWay" + isGivingWay(t) + "Avoid Cross" + isAvoidingCrossing(t));
            float distance;

            distance = Vector3.Distance(rb.position, t.position);

            if (distance < transform.InverseTransformDirection(rb.velocity).magnitude * 3 || distance < 25)
            {
                if (t.GetComponent<DynamicObject>().sailboat && !isAvoidingCrossing(t))
                {
                    //AddReward(Mathf.Min(reward*3, -0.03f));
                    Debug.Log("Sailboat Collision Detected");
                }
                else if (!t.GetComponent<DynamicObject>().sailboat && !isAvoidingCrossing(t))
                {
                    var heading = t.GetComponent<Rigidbody>().position - rb.position;
                    var dot = Vector3.Dot(heading, activeShip.GetComponent<Transform>().forward);

                    // Check if the activeShip is in front of the object
                    if (dot > 0)
                    {
                        if (isSupposedToGiveWay(t))
                        {
                            //AddReward(Mathf.Min(reward*3, -0.03f));
                            Debug.Log("Boat Collision Detected");
                        }
                    }
                }
            }
        }

    }

    /// <summary>
    /// Checks if the vessel should be giving way or not based on speed adn then 
    /// </summary>
    /// <param name="interestingObject"></param>
    /// <returns></returns>
    bool isSupposedToGiveWay(Transform interestingObject)
    {
        double avelActive = transform.InverseTransformDirection(rb.velocity).magnitude;
        Rigidbody rbObj = interestingObject.GetComponent<Rigidbody>();
        double avelObject = rbObj.GetComponent<BoatMovement>().speed;

        //float headingActive = Vector3.SignedAngle(transform.forward, transform.position, Vector3.up);
        //float headingObject = Vector3.SignedAngle(transform.forward, interestingObject.position, Vector3.up);

        Vector3 activePosition = rb.position;
        Vector3 objPosition = rbObj.position;

        if (avelObject < avelActive && !IsGivingWay(activeShip.GetComponent<Transform>(), interestingObject))
        {
            return true;
        }
        return false;
    }

    public bool IsGivingWay(Transform giveWayVessel, Transform standOnVessel)
    {
        // Get the velocities of the vessels
        Vector3 giveWayVelocity = giveWayVessel.GetComponent<Rigidbody>().velocity;
        Vector3 standOnVelocity = standOnVessel.GetComponent<Rigidbody>().velocity;

        // Get the positions of the vessels
        Vector3 giveWayPosition = giveWayVessel.transform.position;
        Vector3 standOnPosition = standOnVessel.transform.position;

        // Calculate the relative velocity between the vessels
        Vector3 relativeVelocity = giveWayVelocity - standOnVelocity;

        // Calculate the relative position between the vessels
        Vector3 relativePosition = standOnPosition - giveWayPosition;

        // Calculate the vector perpendicular to the relative position
        Vector3 perpendicularVector = new Vector3(-relativePosition.z, 0, relativePosition.x).normalized;

        // Calculate the dot product of the relative velocity and the perpendicular vector
        float dotProduct = Vector3.Dot(relativeVelocity, perpendicularVector);

        // If the dot product is positive, the give-way vessel is giving way
        return dotProduct > 0;
    }

    bool isAvoidingCrossing(Transform interestingObject)
    {
        Vector3 path1Start = transform.position;
        Vector3 path1End = path1Start + rb.velocity * 20;

        Rigidbody rbObj = interestingObject.GetComponent<Rigidbody>();
        Vector3 path2Start = rbObj.position;
        Vector3 path2End = path2Start + interestingObject.forward * rbObj.GetComponent<BoatMovement>().speed * 20;

        //Debug.Log(path1Start + " " + path1End + " " + path2Start + " " + path2End);

        return (!Intersect(path1Start, path1End, path2Start, path2End));

        // Check if the agent's path will intersect the other vessel's path
        //if (Vector3.Distance(this.futurePosition(10f), otherVessel.futurePosition(10f)) < 30f)
        //{
        //    // Check if the agent is on a course that will avoid crossing the other vessel's path
        //    if (Mathf.Abs(Vector3.Angle(this.heading, otherVessel.heading)) < 90)
        //    {
        //        return true;
        //    }
        //}
    }

    /// <summary>
    /// Returns if the two lines intersect or not
    /// </summary>
    /// <param name="line1Start"></param>
    /// <param name="line1End"></param>
    /// <param name="line2Start"></param>
    /// <param name="line2End"></param>
    /// <returns></returns>
    bool Intersect(Vector3 line1Start, Vector3 line1End, Vector3 line2Start, Vector3 line2End)
    {
        Vector3 line1 = line1End - line1Start;
        Vector3 line2 = line2End - line2Start;
        Vector3 cross = Vector3.Cross(line1, line2);

        if (cross.sqrMagnitude < float.Epsilon)
        {
            return false;
        }

        Vector3 direction = line2Start - line1Start;
        float t = Vector3.Cross(direction, line2).magnitude / cross.magnitude;
        float u = Vector3.Cross(direction, line1).magnitude / cross.magnitude;

        return (t >= 0 && t <= 1 && u >= 0 && u <= 1);
    }
}

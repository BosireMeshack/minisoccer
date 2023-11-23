using System.Collections;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEditor.Timeline.Actions;
using UnityEngine;
using UnityEngine.EventSystems;

public class player : Agent
{
    public float ballControlRadius = 1.5f;
    public Rigidbody ballRigidbody;
    public float ballControlFov = 45.0f;
    public Rigidbody rb;
    public GameObject ball;
    public GameObject goal;
    public float kickForce = 0.1f;

    public float speed = 1f;

    public float rotationSpeed = 1f;

    // Environemtn parameters
    public float environmentSizeX  = 16;
    public float environmentSizeZ  = 8;
    //public float boundaryRadius = 1f;

    // private Vector3 startingPosition = new Vector3(0.0f, 0.0f, 0.0f);

    // private enum ACTIONS {
    //     NOTHING = 0,
    //     KICKBALL = 1,
        
    // }

    public override void OnEpisodeBegin()
    {
        // Reset the environment at the beginning of each episode
        ball.transform.localPosition = new Vector3(4f, 0f, 0f);
        //ball.GetComponent<Rigidbody>().velocity = Vector3.zero;
        // ball.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

         // Randomly place the agent on the environment
        transform.localPosition = GetRandomPositionInBounds();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        //Direction of the ball
        Vector3 directionBall = (ball.transform.localPosition - transform.localPosition).normalized;
        
        sensor.AddObservation(directionBall.x);
        sensor.AddObservation(directionBall.z);

        //Direction of the goal
        Vector3 directionGoal = (goal.transform.localPosition - transform.localPosition).normalized;
        sensor.AddObservation(directionGoal.x);
        sensor.AddObservation(directionGoal.z);
       
        // Direction of the agent
        sensor.AddObservation(transform.forward);

        
    }

    
    

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Normal movement based on actions
        var actionTaken = actions.ContinuousActions;
        // Kicking the ball based on discrete actions
        var kickAction = actions.DiscreteActions[0];

        // Use index 0 for forward movement and index 1 for turning
        float moveDirection = actionTaken[0];
        float turnDirection = actionTaken[1];

        Vector3 moveVector = transform.forward * moveDirection * speed;
        rb.MovePosition(rb.position + moveVector * Time.fixedDeltaTime);

        float rotation = turnDirection * rotationSpeed * Time.fixedDeltaTime;
        rb.MoveRotation(rb.rotation * Quaternion.Euler(Vector3.up * rotation));

        // Check if kick action is taken (assuming 1 is the kick action index)
        if (kickAction == 1)
        {
            KickBall();
        }
        

        
    }

    // Separate method to handle kicking the ball
    private void KickBall()
    {
        // float kickRange = 45f;
        // // Implement kicking logic here
        // // For example, apply force to kick the ball
        // if (Vector3.Distance(transform.position, ball.transform.position) < kickRange)
        // {
        //     Vector3 kickDirection = (ball.transform.position - transform.position).normalized;
        //     rb.AddForce(kickDirection * kickForce, ForceMode.Impulse);
        // }

        if ((ball.transform.localPosition - transform.localPosition).magnitude < ballControlRadius) {
                    Vector3 directionToBall = (ball.transform.localPosition - transform.localPosition);
                    directionToBall.Normalize();
                    float angleBetweenPlayerAndBall = Mathf.Rad2Deg * Mathf.Acos(Mathf.Clamp(Vector3.Dot(transform.forward, directionToBall), 0.0f, 1.0f));
                    bool isColliderInFront = (angleBetweenPlayerAndBall < ballControlFov);
                    if (isColliderInFront)
                    {
                        Debug.Log("Kicking the ball!");
                        ballRigidbody.AddForce(directionToBall * kickForce);
                    }
                        
                    
        }
    }
    

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> actions = actionsOut.ContinuousActions;
        ActionSegment<int> discreteActions = actionsOut.DiscreteActions;
        // Control forward and backward movement
        float moveInput = 0;
        if (Input.GetKey("w")) moveInput = 1;
        else if (Input.GetKey("s")) moveInput = -1;

        // Control turning
        float turnInput = 0;
        if (Input.GetKey("d")) turnInput = 1;
        else if (Input.GetKey("a")) turnInput = -1;

        actions[0] = moveInput;
        actions[1] = turnInput;

        // Kick the ball when the space key is pressed (assuming 1 is the kick action index)
        if (Input.GetKeyDown(KeyCode.Space))
        {
            discreteActions[0] = 1;
        }
        else
        {
            discreteActions[0] = 0;
        }
        
    }


    
    public void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.tag == "ball")
        {
            
            AddReward(0.5f);
            Debug.Log("COLLIDED WITH BALL");
            

        }

        
        if (collision.collider.tag == "wall")
        {
            AddReward(-1f);
            EndEpisode();
            Debug.Log("COLLIDED WITH WALL");

        }

        if (collision.collider.tag == "goaline")
        {
            AddReward(-1f);
            EndEpisode();
            Debug.Log("COLLIDED WITH THE GOALINE");
        }
        if (collision.collider.tag == "goal")
        {
            AddReward(-1f);
            EndEpisode();
            Debug.Log("COLLIDED WITH THE GOALPOST");
        }
        // if (collision.ballRigidbody.CompareTag("goaline"))
        // {
        //     // Handle the collision logic here
        //     Debug.Log("Collision detected between goaline and ball");
        // }

       



    }

    public void HandleGoalCollision()
    {
        AddReward(1f);
        EndEpisode();
    }

    public void HandleWallCollision()
    {
        AddReward(-1);
        EndEpisode();
    }
    
    
      private Vector3 GetRandomPositionInBounds()
    {
        // Get a random position within the circular bounds
        
        float x = Random.Range(-environmentSizeX / 2, environmentSizeX / 2);
        float z = Random.Range(-environmentSizeZ / 2, environmentSizeZ / 2);
        return new Vector3(x, -0.6f, z);
        

        
    }
}

using UnityEngine;

public class BallCollision : MonoBehaviour
{
    public player agent;
    private void OnCollisionEnter(Collision collision)
    {
        if (collision.collider.CompareTag("wall"))
        {
            // Handle collision with the wall
            Debug.Log("Ball collided with the wall");

            agent.HandleWallCollision();
            
            
        }
        else if (collision.collider.CompareTag("goaline"))
        {
            // Handle collision with the goalpost 
            Debug.Log("Ball collided with the goaline");
            
            agent.HandleGoalCollision();
            
        }

        // You can add more collision checks here if necessary
    }
}
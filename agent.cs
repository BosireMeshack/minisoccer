// using UnityEngine;

// public class ForceAdder : MonoBehaviour
// {
//     public Rigidbody rb;
//     [SerializeField]
//     private Vector3 forceDirection = Vector3.forward;

//     [SerializeField]
//     private float forceMagnitude = 10f;

//     [SerializeField]
//     private ForceMode forceMode = ForceMode.Force;

//     void Update()
//     {
//         if (Input.GetKeyDown(KeyCode.Space))
//         {
//             AddForce();
//         }
//     }

//     public void AddForce()
//     {
        
//         if (rb != null)
//         {
//             rb.AddForce(forceDirection.normalized * forceMagnitude, forceMode);
//         }
//         else
//         {
//             Debug.LogWarning("Rigidbody component not found on the GameObject.");
//         }
//     }
// }
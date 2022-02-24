using System.Collections;
using System.Collections.Generic;
using UnityEngine;

// Put this script on the lower leg components of the
// spot to detect if it is touching the ground
public class FootCollisionSensor : MonoBehaviour
{
    public bool colliding = false; 

    // Foot is touching something
    void OnCollisionEnter()
    {
        colliding = true;
    }
    // Foot is not touching something
    void OnCollisionExit()
    {
        colliding = false; 
    }
}

using UnityEngine;
using System;

public class SpotControllerInput : MonoBehaviour
{
    private SpotJointPositionSubscriber spotjointsubscriber;
    private SpotController spotController;

    void Start()
    {
        spotController = GetComponent<SpotController>();
        spotjointsubscriber = GetComponent<SpotJointPositionSubscriber>();
    }


    void Update()
    {
        for (int i = 0; i < spotController.joints.Length; i++)
        {
            // Convert the joint values from radians to degrees
            float inputVal = spotjointsubscriber.jointPositions[i] * 180.0f / (float)Math.PI;

            if (Mathf.Abs(inputVal) > 0)
            {
                // Update rotation goal
                spotController.UpdateRotation(i, SpotState.Go, inputVal);
            }
        }
        //robotController.StopAllJointRotations();
    }
}

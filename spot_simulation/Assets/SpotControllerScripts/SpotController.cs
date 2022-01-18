using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class SpotController : MonoBehaviour
{
    [System.Serializable]
    public struct Joint
    {
        public string inputAxis;
        public GameObject robotPart;
    }
    public Joint[] joints;

    private ArticulationJointController[] controllers;

    void Start()
    {
        // Get all joints
        controllers = new ArticulationJointController[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            controllers[i] = joints[i].robotPart.GetComponent<ArticulationJointController>();
        }
    }

    // Stop the joints if necessary
    public void StopAllJointRotations()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            UpdateRotation(i, SpotState.Stop, 0f);
        }
    }

    // Updates rotation goal for controller
    public void UpdateRotation(int jointIndex, SpotState state, float jointPosition)
    {
        controllers[jointIndex].rotationGoal = jointPosition;
        controllers[jointIndex].robotState = state;
    }




}

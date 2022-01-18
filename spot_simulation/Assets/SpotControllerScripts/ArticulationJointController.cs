using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public enum SpotState { Stop = 0, Go = 1 };

public class ArticulationJointController : MonoBehaviour
{
    public SpotState robotState = SpotState.Stop;
    public float rotationGoal;
    public float speed = 100.0f;

    private ArticulationBody articulation;


    // LIFE CYCLE

    void Start()
    {
        articulation = GetComponent<ArticulationBody>();
    }

    void FixedUpdate()
    {
        if (robotState != SpotState.Stop)
        {
            RotateTo(rotationGoal);
        }
    }

    // MOVEMENT HELPERS

/*    float CurrentPrimaryAxisRotation()
    {
        float currentRotationRads = articulation.jointPosition[0];
        float currentRotation = Mathf.Rad2Deg * currentRotationRads;
        return currentRotation;
    }*/

    // Provide drive values to rotate to new goal
    void RotateTo(float primaryAxisRotation)
    {
        var drive = articulation.xDrive;
        drive.target = primaryAxisRotation;
        articulation.xDrive = drive;
    }
}

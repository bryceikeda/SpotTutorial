using System;
using Unity.Robotics;
using UnityEngine;

public class JointAttributes : MonoBehaviour
{
    private ArticulationBody[] articulationChain;
    // Stores original colors of the part being highlighted
    private Color[] prevColor;
    private int previousIndex;

    [InspectorReadOnly(hideInEditMode: true)]
    public string selectedJoint;
    [HideInInspector]
    public int selectedIndex;

    public float stiffness;
    public float damping;
    public float forceLimit;
    //public float speed = 5f; // Units: degree/s
    //public float torque = 100f; // Units: Nm or N
    //public float acceleration = 5f;// Units: m/s^2 / degree/s^2

    public static readonly string[] JointNames =
    { "rear_right_lower_leg", "rear_left_lower_leg", "front_left_lower_leg", "front_right_lower_leg", "rear_right_hip",  "rear_left_hip",
        "front_right_hip", "front_left_hip", "rear_right_upper_leg", "rear_left_upper_leg", "front_left_upper_leg", "front_right_upper_leg"};
    // Start is called before the first frame update
    void Start()
    {
        previousIndex = selectedIndex = 1;
        articulationChain = this.GetComponentsInChildren<ArticulationBody>();
        int defDyanmicVal = 10;
        foreach (ArticulationBody joint in articulationChain)
        {
            for (int i = 0; i < JointNames.Length; i++)
            {
                if (joint.gameObject.name == JointNames[i])
                {
                    joint.jointFriction = defDyanmicVal;
                    joint.angularDamping = defDyanmicVal;
                    ArticulationDrive drive = joint.xDrive;
                    drive.stiffness = stiffness;
                    drive.damping = damping;
                    drive.forceLimit = forceLimit;
                    joint.xDrive = drive;
                }
            }

        }

//        JointControl current = articulationChain[jointIndex].GetComponent<JointControl>();

    }

    public void UpdateControlType(JointControl joint)
    {

        
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Float = RosMessageTypes.Std.Float64Msg;

public class SpotJointSubscriber : MonoBehaviour
{
    // Each link name
    public static readonly string[] LinkNames =
    {   "base_link/front_rail/front_left_hip",
        "base_link/front_rail/front_left_hip/front_left_upper_leg",
        "base_link/front_rail/front_left_hip/front_left_upper_leg/front_left_lower_leg",
        "base_link/front_rail/front_right_hip",
        "base_link/front_rail/front_right_hip/front_right_upper_leg",
        "base_link/front_rail/front_right_hip/front_right_upper_leg/front_right_lower_leg",
        "base_link/rear_rail/rear_left_hip",
        "base_link/rear_rail/rear_left_hip/rear_left_upper_leg",
        "base_link/rear_rail/rear_left_hip/rear_left_upper_leg/rear_left_lower_leg",
        "base_link/rear_rail/rear_right_hip",
        "base_link/rear_rail/rear_right_hip/rear_right_upper_leg",
        "base_link/rear_rail/rear_right_hip/rear_right_upper_leg/rear_right_lower_leg"
    };

    // Hardcoded variables
    const int k_NumRobotJoints = 12;

    [SerializeField]
    GameObject m_Spot;
    public GameObject Spot { get => m_Spot; set => m_Spot = value; }

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    ROSConnection m_Ros;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();

        // Create array for articulation bodies of each joint
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];

        // Get the articulationbody for each joint
        var linkName = string.Empty;
        for (var i = 0; i < k_NumRobotJoints; i++)
        {
            m_JointArticulationBodies[i] = m_Spot.transform.Find(LinkNames[i]).GetComponent<ArticulationBody>();
        }

        // Subscribe to each command topic
        m_Ros.Subscribe<Float>("spot1/joint_front_left_hip_x_controller/command", UpdateFrontLeftHipX);
        m_Ros.Subscribe<Float>("spot1/joint_front_left_hip_y_controller/command", UpdateFrontLeftHipY);
        m_Ros.Subscribe<Float>("spot1/joint_front_left_knee_controller/command", UpdateFrontLeftKnee);
        m_Ros.Subscribe<Float>("spot1/joint_front_right_hip_x_controller/command", UpdateFrontRightHipX);
        m_Ros.Subscribe<Float>("spot1/joint_front_right_hip_y_controller/command", UpdateFrontRightHipY);
        m_Ros.Subscribe<Float>("spot1/joint_front_right_knee_controller/command", UpdateFrontRightKnee);
        m_Ros.Subscribe<Float>("spot1/joint_rear_left_hip_x_controller/command", UpdateRearLeftHipX);
        m_Ros.Subscribe<Float>("spot1/joint_rear_left_hip_y_controller/command", UpdateRearLeftHipY);
        m_Ros.Subscribe<Float>("spot1/joint_rear_left_knee_controller/command", UpdateRearLeftKnee);
        m_Ros.Subscribe<Float>("spot1/joint_rear_right_hip_x_controller/command", UpdateRearRightHipX);
        m_Ros.Subscribe<Float>("spot1/joint_rear_right_hip_y_controller/command", UpdateRearRightHipY);
        m_Ros.Subscribe<Float>("spot1/joint_rear_right_knee_controller/command", UpdateRearRightKnee);


    }

    // Update the joint angle by setting the
    // xDrive.Target of each Articulationbody
    public void UpdateJointAngle(double cmd, int joint)
    {
        var angle = (float)cmd * Mathf.Rad2Deg;
        var jointXDrive = m_JointArticulationBodies[joint].xDrive;
        jointXDrive.target = angle;
        m_JointArticulationBodies[joint].xDrive = jointXDrive;
    }

    public void UpdateFrontLeftHipX(Float cmd)
    {
        UpdateJointAngle(cmd.data, 0);
    }

    public void UpdateFrontLeftHipY(Float cmd)
    {
        UpdateJointAngle(cmd.data, 1);
    }

    public void UpdateFrontLeftKnee(Float cmd)
    {
        UpdateJointAngle(cmd.data, 2);
    }

    public void UpdateFrontRightHipX(Float cmd)
    {
        UpdateJointAngle(cmd.data, 3);
    }

    public void UpdateFrontRightHipY(Float cmd)
    {
        UpdateJointAngle(cmd.data, 4);
    }

    public void UpdateFrontRightKnee(Float cmd)
    {
        UpdateJointAngle(cmd.data, 5);
    }

    public void UpdateRearLeftHipX(Float cmd)
    {
        UpdateJointAngle(cmd.data, 6);
    }

    public void UpdateRearLeftHipY(Float cmd)
    {
        UpdateJointAngle(cmd.data, 7);
    }

    public void UpdateRearLeftKnee(Float cmd)
    {
        UpdateJointAngle(cmd.data, 8);
    }

    public void UpdateRearRightHipX(Float cmd)
    {
        UpdateJointAngle(cmd.data, 9);
    }

    public void UpdateRearRightHipY(Float cmd)
    {
        UpdateJointAngle(cmd.data, 10);
    }

    public void UpdateRearRightKnee(Float cmd)
    {
        UpdateJointAngle(cmd.data, 11);
    }


}

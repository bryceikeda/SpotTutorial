using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using Float = RosMessageTypes.Std.Float64Msg; 

public class SpotJointPositionSubscriber : MonoBehaviour
{
    private float frontLeftHipXJointPos = 0.0f;
    private float frontLeftHipYJointPos = 0.0f;
    private float frontLeftKneeJointPos = 0.0f;
    private float frontRightHipXJointPos = 0.0f;
    private float frontRightHipYJointPos = 0.0f;
    private float frontRightKneeJointPos = 0.0f;
    private float rearLeftHipXJointPos = 0.0f;
    private float rearLeftHipYJointPos = 0.0f;
    private float RearLeftKneeJointPos = 0.0f;
    private float rearRightHipXJointPos = 0.0f;
    private float rearRightHipYJointPos = 0.0f;
    private float rearRightKneeJointPos = 0.0f;

    public float[] jointPositions;

    ROSConnection m_Ros;

    void Start()
    {
        // Subscribe to each joint command topic 
        m_Ros = ROSConnection.instance; 
        m_Ros.Subscribe<Float>("/spot1/joint_front_left_hip_x_controller/command", FrontLeftHipX);
        m_Ros.Subscribe<Float>("/spot1/joint_front_left_hip_y_controller/command", FrontLeftHipY);
        m_Ros.Subscribe<Float>("/spot1/joint_front_left_knee_controller/command", FrontLeftKnee);
        m_Ros.Subscribe<Float>("/spot1/joint_front_right_hip_x_controller/command", FrontRightHipX);
        m_Ros.Subscribe<Float>("/spot1/joint_front_right_hip_y_controller/command", FrontRightHipY);
        m_Ros.Subscribe<Float>("/spot1/joint_front_right_knee_controller/command", FrontRightKnee);
        m_Ros.Subscribe<Float>("/spot1/joint_rear_left_hip_x_controller/command", RearLeftHipX);
        m_Ros.Subscribe<Float>("/spot1/joint_rear_left_hip_y_controller/command", RearLeftHipY);
        m_Ros.Subscribe<Float>("/spot1/joint_rear_left_knee_controller/command", RearLeftKnee);
        m_Ros.Subscribe<Float>("/spot1/joint_rear_right_hip_x_controller/command", RearRightHipX);
        m_Ros.Subscribe<Float>("/spot1/joint_rear_right_hip_y_controller/command", RearRightHipY);
        m_Ros.Subscribe<Float>("/spot1/joint_rear_right_knee_controller/command", RearRightKnee);

        jointPositions = new float[12];
    }

    // Update is called once per frame
    void FixedUpdate()
    {

        // This needs to match the robot controller script joint order
        // Update joint positions
        jointPositions[0] = frontLeftHipXJointPos;
        jointPositions[1] = frontLeftHipYJointPos;
        jointPositions[2] = frontLeftKneeJointPos;
        jointPositions[3] = frontRightHipXJointPos;
        jointPositions[4] = frontRightHipYJointPos;
        jointPositions[5] = frontRightKneeJointPos;
        jointPositions[6] = rearLeftHipXJointPos;
        jointPositions[7] = rearLeftHipYJointPos;
        jointPositions[8] = RearLeftKneeJointPos;
        jointPositions[9] = rearRightHipXJointPos;
        jointPositions[10] = rearRightHipYJointPos;
        jointPositions[11] = rearRightKneeJointPos;


        /*
        jointPos_frontLeftHip = 0;
        jointPos_frontLeftUpperLeg = 0;
        jointPos_frontLeftLowerLeg = 0;
        jointPos_frontRightHip = 0;
        jointPos_frontRightUpperLeg = 0;
        jointPos_frontRightLowerLeg = 0;
        jointPos_rearLeftHip = 0;
        jointPos_rearLeftUpperLeg = 0;
        jointPos_rearLeftLowerLeg = 0;
        jointPos_rearRightHip = 0;
        jointPos_rearRightUpperLeg = 0;
        jointPos_rearRightLowerLeg = 0;*/
    }

    public void FrontLeftHipX(Float jointPosCommandValue)
    {
        frontLeftHipXJointPos = (float)jointPosCommandValue.data;
    }

    public void FrontLeftHipY(Float jointPosCommandValue)
    {
        frontLeftHipYJointPos = (float)jointPosCommandValue.data;
    }

    public void FrontLeftKnee(Float jointPosCommandValue)
    {
        frontLeftKneeJointPos = (float)jointPosCommandValue.data;
    }

    public void FrontRightHipX(Float jointPosCommandValue)
    {
        frontRightHipXJointPos= (float)jointPosCommandValue.data;
    }

    public void FrontRightHipY(Float jointPosCommandValue)
    {
        frontRightHipYJointPos = (float)jointPosCommandValue.data;
    }

    public void FrontRightKnee(Float jointPosCommandValue)
    {
        frontRightKneeJointPos = (float)jointPosCommandValue.data;
    }

    public void RearLeftHipX(Float jointPosCommandValue)
    {
        rearLeftHipXJointPos = (float)jointPosCommandValue.data;
    }

    public void RearLeftHipY(Float jointPosCommandValue)
    {
        rearLeftHipYJointPos = (float)jointPosCommandValue.data;
    }

    public void RearLeftKnee(Float jointPosCommandValue)
    {
        RearLeftKneeJointPos = (float)jointPosCommandValue.data;
    }

    public void RearRightHipX(Float jointPosCommandValue)
    {
        rearRightHipXJointPos = (float)jointPosCommandValue.data;
    }

    public void RearRightHipY(Float jointPosCommandValue)
    {
        rearRightHipYJointPos = (float)jointPosCommandValue.data;
    }

    public void RearRightKnee(Float jointPosCommandValue)
    {
        rearRightKneeJointPos = (float)jointPosCommandValue.data;
    }
}

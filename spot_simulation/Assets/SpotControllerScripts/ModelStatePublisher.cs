using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using RosMessageTypes.Gazebo;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;


// Publish the model states so the spot controller package
// Knows there is a spot in the scene
public class ModelStatePublisher : MonoBehaviour
{
    [SerializeField]
    GameObject m_Spot;

    // ROS Connector
    ROSConnection m_Ros;

    [SerializeField]
    string rosModelStatesTopicName = "gazebo/model_states";
    [SerializeField]
    private string m_RobotName = "spot1";

    float updateRate = 1f;
    float counter;

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<ModelStatesMsg>(rosModelStatesTopicName);

        counter = Time.fixedTime + updateRate;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (Time.fixedTime >= counter)
        {
            PublishModelState();
            counter = Time.fixedTime + updateRate;
        }
    }

    // Update the position and orientation of the 
    // Spot robot and publish it
    public void PublishModelState()
    {
        // Create the spot modelstate message, there's
        // probably a correct way to do this
        string[] name = new string[1] { m_RobotName };

        PoseMsg[] poseMsg = new PoseMsg[1];
        poseMsg[0] = new PoseMsg()
        {
            position = m_Spot.transform.position.To<FLU>(),
            orientation = m_Spot.transform.rotation.To<FLU>()
        };

        TwistMsg[] twistMsg = new TwistMsg[1];
        twistMsg[0] = new TwistMsg()
        {
            linear = new Vector3Msg(),
            angular = new Vector3Msg()
        };

        var model_states = new ModelStatesMsg(name, poseMsg, twistMsg);

        m_Ros.Publish(rosModelStatesTopicName, model_states);
    }
}

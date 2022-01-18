using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Float = RosMessageTypes.Std.Float64Msg;

public class ClockPublisher : MonoBehaviour
{
    // ROS Connector
    ROSConnection m_Ros;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.instance;
        m_Ros.RegisterPublisher<Float>("/unity/time");
    }

    // Update is called once per frame
    void Update()
    {
        Float unityTime= new Float(Time.time);
        m_Ros.Send("/unity/time", unityTime); 
    }
}

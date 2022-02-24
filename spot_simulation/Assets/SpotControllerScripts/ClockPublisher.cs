using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.BuiltinInterfaces;

public class ClockPublisher : MonoBehaviour
{
    // ROS Connector
    ROSConnection m_Ros;

    // Variables required for ROS communication
    public string rosClockTopicName = "clock";

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.RegisterPublisher<TimeMsg>(rosClockTopicName);
    }

    // Publish simulated clock
    void FixedUpdate()
    {
        var clock = new TimeMsg
        {
            sec = (uint)Time.realtimeSinceStartup,
            nanosec = 0
        };
        m_Ros.Publish(rosClockTopicName, clock);
    }
}

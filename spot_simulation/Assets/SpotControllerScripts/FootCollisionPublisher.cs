using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using RosMessageTypes.Gazebo;


// Send the foot collision data to ROS for the spot_controller
public class FootCollisionPublisher : MonoBehaviour
{
    // Name of each Unity Spot foot 
    public static readonly string[] FeetNames =
    {   "base_link/front_rail/front_left_hip/front_left_upper_leg/front_left_lower_leg",
        "base_link/front_rail/front_right_hip/front_right_upper_leg/front_right_lower_leg",
        "base_link/rear_rail/rear_left_hip/rear_left_upper_leg/rear_left_lower_leg",
        "base_link/rear_rail/rear_right_hip/rear_right_upper_leg/rear_right_lower_leg"
    };
    
    // Names of each foot topic
    public static readonly string[] rosFeetNameTopics =
    {   "front_left_lower_leg_contact",
        "front_right_lower_leg_contact",
        "rear_left_lower_leg_contact",
        "rear_right_lower_leg_contact" 
    };

    [SerializeField]
    GameObject m_Spot;
    // ROS Connector
    ROSConnection m_Ros;

    FootCollisionSensor[] m_FootCollisionSensors;

    const int k_NumFeetContacts = 4;
    string m_RobotName = "spot1";

    float updateRate = .1f;
    float counter;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();

        counter = Time.fixedTime + updateRate;

        // Get each food FootCollisionSensor script and register the publisher
        m_FootCollisionSensors = new FootCollisionSensor[k_NumFeetContacts];
        for (int i = 0; i < k_NumFeetContacts; i++)
        {
            m_Ros.RegisterPublisher<ContactsStateMsg>(m_RobotName + "/" + rosFeetNameTopics[i]);
            m_FootCollisionSensors[i] = m_Spot.transform.Find(FeetNames[i]).GetComponent<FootCollisionSensor>();
        }

    }

    void Update()
    {
        if (Time.fixedTime >= counter)
        {
            PublishContactStates();
            counter = Time.fixedTime + updateRate;
        }
    }

    // Send the feet contacts to ROS
    public void PublishContactStates()
    {
        for (int i = 0; i < k_NumFeetContacts; i++)
        {
            if (m_FootCollisionSensors[i].colliding)
            {
                // Create the spot footcontact message, there's
                // probably a correct way to do this
                var header = new HeaderMsg();

                var contact_state = new ContactStateMsg[1];
                contact_state[0] = new ContactStateMsg();

                var contacts_state = new ContactsStateMsg(header, contact_state);

                m_Ros.Publish(m_RobotName + "/" + rosFeetNameTopics[i], contacts_state);
            }
            else
            {
                m_Ros.Publish(m_RobotName + "/" + rosFeetNameTopics[i], new ContactsStateMsg());
            }
        }
        
    }

}

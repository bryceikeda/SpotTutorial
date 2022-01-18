using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using ContactsState = RosMessageTypes.Gazebo.ContactsStateMsg;
using ContactState = RosMessageTypes.Gazebo.ContactStateMsg;
using Float = RosMessageTypes.Std.Float64Msg;


public class FootCollisionPublisher : MonoBehaviour
{
    [SerializeField]
    private string robotName = "spot1";

    [SerializeField]
    GameObject m_Spot;
    // ROS Connector
    public ROSConnection m_Ros;

    public static readonly string[] FeetNames =
        { "rear_right_lower_leg", "rear_left_lower_leg", "front_left_lower_leg", "front_right_lower_leg" };
    
    public static readonly string[] FeetNameTopics =
        { "/rear_right_lower_leg_contact", "/rear_left_lower_leg_contact", "/front_left_lower_leg_contact", "/front_right_lower_leg_contact" };


    FootCollisionSensor[] feetContacts;
    const int k_NumFeetContacts = 4;

    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.instance;
        m_Spot = GameObject.Find(robotName);

        feetContacts = new FootCollisionSensor[k_NumFeetContacts];

        if (m_Spot == null)
        {
            Debug.LogError("No robot named" + robotName + "found.");
        }

        for (int i = 0; i < FeetNames.Length; i++)
        {
            //m_Ros.RegisterPublisher<ContactsState>("/" + robotName + FeetNameTopics[i]);
            feetContacts = m_Spot.GetComponentsInChildren<FootCollisionSensor>();
        }
    }

    // Update is called once per frame
    void Update()
    {
        PublishContactStates(); 
    }

    public void PublishContactStates()
    {   
        var frontLeftLowerLeg = new ContactsState();
        var frontRightLowerLeg = new ContactsState();
        var rearLeftLowerLeg = new ContactsState();
        var rearRightLowerLeg = new ContactsState();


        // for each leg, get if it is touching the ground 
        for (int i = 0; i < feetContacts.Length; i++)
        {
            if (feetContacts[i].gameObject.name == "rear_right_lower_leg")
            {
                if (feetContacts[i].colliding == true)
                {
                    rearRightLowerLeg.states = new ContactState[1];
                }
            }
            else if (feetContacts[i].gameObject.name == "rear_left_lower_leg")
            {
                if (feetContacts[i].colliding == true)
                {
                    rearLeftLowerLeg.states = new ContactState[1];
                }
            }
            else if (feetContacts[i].gameObject.name == "front_left_lower_leg")
            {
                if (feetContacts[i].colliding == true) 
                {
                    frontLeftLowerLeg.states = new ContactState[1];
                }
            }
            else if (feetContacts[i].gameObject.name == "front_right_lower_leg")
            {
                if (feetContacts[i].colliding == true)
                {
                    frontRightLowerLeg.states = new ContactState[1];
                }
            }
        }

       // m_Ros.Send("/" + robotName + FeetNameTopics[0], rearRightLowerLeg);
      //  m_Ros.Send("/" + robotName + FeetNameTopics[1], rearLeftLowerLeg);
      //  m_Ros.Send("/" + robotName + FeetNameTopics[2], frontLeftLowerLeg);
     //   m_Ros.Send("/" + robotName + FeetNameTopics[3], frontRightLowerLeg);
    }

}

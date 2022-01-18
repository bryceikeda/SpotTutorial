using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Std;
using ModelStates = RosMessageTypes.Gazebo.ModelStatesMsg;
using RosMessageTypes.Geometry;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using FloatArray = RosMessageTypes.Std.Float64MultiArrayMsg; 

public class ModelStatePublisher : MonoBehaviour
{
    [SerializeField]
    private string robotName = "spot1";

    [SerializeField]
    GameObject m_Spot;
    // ROS Connector
    public ROSConnection m_Ros;
    public GameObject connection; 
    string modelStatesTopic = "/gazebo/model_states";

    ModelStates modelstates;

    //  broadcast all model states in world frame
    public string[] name;
    //  model names
    public PoseMsg[] pose;
    //  desired pose in world frame
    public TwistMsg[] twist;

    private FloatArray modelArr;
    private MultiArrayLayoutMsg layout;
    // Start is called before the first frame update
    void Start()
    {
        // Get ROS connection static instance
        m_Ros = connection.GetComponent<ROSConnection>();
        m_Spot = GameObject.Find(robotName);
        //m_Ros.RegisterPublisher<ModelStates>(modelStatesTopic);

        name = new string[1];
        pose = new PoseMsg[1];
        twist = new TwistMsg[1];
        modelstates = new ModelStates(name, pose, twist);
        layout = new MultiArrayLayoutMsg(); 
        modelArr = new FloatArray(layout, new double[7]); 
        // robot name needs to be added in the file via ROS
        modelstates.name[0] = robotName;
    }

    // Update is called once per frame
    void Update()
    {
        UpdatePose();
        PublishModelState(); 
    }

    public void PublishModelState()
    {
        m_Ros.Send(modelStatesTopic, modelstates);
    }

    public void UpdatePose()
    {
        var pos = m_Spot.transform.position.To<FLU>();
        var ori = m_Spot.transform.rotation.To<FLU>();

        modelArr.data[0] = pos.x;
        modelArr.data[1] = pos.y;
        modelArr.data[2] = pos.z;
        modelArr.data[3] = ori.x;
        modelArr.data[4] = ori.y;
        modelArr.data[5] = ori.z;
        modelArr.data[6] = ori.w;


        modelstates.pose[0] = new PoseMsg
        { 
            position = m_Spot.transform.position.To<FLU>(),
            orientation = m_Spot.transform.rotation.To<FLU>()
        };
    }

}

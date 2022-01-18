#!/usr/bin/env python
import rospy
from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService
from std_msgs.msg import Float64, Float64MultiArray
from gazebo_msgs.msg import ContactState, ContactsState, ModelStates, ModelState



def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections)
    rospy.init_node(ros_node_name, anonymous=True)
    
    tcp_server.start({
        '/spot1/joint_front_left_hip_x_controller/command': RosSubscriber('/spot1/joint_front_left_hip_x_controller/command', Float64, tcp_server),
        '/spot1/joint_front_left_hip_y_controller/command': RosSubscriber('/spot1/joint_front_left_hip_y_controller/command', Float64, tcp_server),
        '/spot1/joint_front_left_knee_controller/command': RosSubscriber('/spot1/joint_front_left_knee_controller/command', Float64, tcp_server),
        '/spot1/joint_front_right_hip_x_controller/command': RosSubscriber('/spot1/joint_front_right_hip_x_controller/command', Float64, tcp_server),
        '/spot1/joint_front_right_hip_y_controller/command': RosSubscriber('/spot1/joint_front_right_hip_y_controller/command', Float64, tcp_server),
        '/spot1/joint_front_right_knee_controller/command': RosSubscriber('/spot1/joint_front_right_knee_controller/command', Float64, tcp_server),
        '/spot1/joint_rear_left_hip_x_controller/command': RosSubscriber('/spot1/joint_rear_left_hip_x_controller/command', Float64, tcp_server),
        '/spot1/joint_rear_left_hip_y_controller/command': RosSubscriber('/spot1/joint_rear_left_hip_y_controller/command', Float64, tcp_server),
        '/spot1/joint_rear_left_knee_controller/command': RosSubscriber('/spot1/joint_rear_left_knee_controller/command', Float64, tcp_server),
        '/spot1/joint_rear_right_hip_x_controller/command': RosSubscriber('/spot1/joint_rear_right_hip_x_controller/command', Float64, tcp_server),
        '/spot1/joint_rear_right_hip_y_controller/command': RosSubscriber('/spot1/joint_rear_right_hip_y_controller/command', Float64, tcp_server),
        '/spot1/joint_rear_right_knee_controller/command': RosSubscriber('/spot1/joint_rear_right_knee_controller/command', Float64, tcp_server), 
        '/gazebo/modelstates' : RosPublisher('/gazebo/modelstates', ModelStates, queue_size=10),
        '/spot1/rear_right_lower_leg' : RosPublisher('/spot1/rear_right_lower_leg', ContactsState, queue_size=10),
        '/spot1/rear_left_lower_leg' : RosPublisher('/spot1/rear_left_lower_leg', ContactsState, queue_size=10),
        '/spot1/front_left_lower_leg' : RosPublisher('/spot1/front_left_lower_leg', ContactsState, queue_size=10),
        '/spot1/front_right_lower_leg' : RosPublisher('/spot1/front_right_lower_leg', ContactsState, queue_size=10)
        })




    rospy.spin()
if __name__ == "__main__":
    main()

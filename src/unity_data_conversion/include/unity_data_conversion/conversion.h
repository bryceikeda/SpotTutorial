#ifndef CONVERSION_H
#define CONVERSION_H

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Float64MultiArray.h"
#include "gazebo_msgs/ContactState.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "rosgraph_msgs/Clock.h"

using namespace std; 

class Conversion{
	public:
		Conversion(); 
		void handle_gazeboModelStates(const std_msgs::Float64MultiArray& msg);
		void handle_rearRightLowerLeg(const std_msgs::Float64& msg);
		void handle_rearLeftLowerLeg(const std_msgs::Float64& msg);
		void handle_frontRightLowerLeg(const std_msgs::Float64& msg);
		void handle_frontLeftLowerLeg(const std_msgs::Float64& msg);
		void handle_time(const std_msgs::Float64& msg);
		ros::Publisher modelstates_publisher; 
		ros::Publisher rearRightLowerLeg_publisher; 
		ros::Publisher rearLeftLowerLeg_publisher; 
		ros::Publisher frontRightLowerLeg_publisher; 
		ros::Publisher frontLeftLowerLeg_publisher; 
		ros::Publisher simtime_publisher;  
		gazebo_msgs::ContactsState rearRightLowerLeg;
		gazebo_msgs::ContactsState rearLeftLowerLeg;
		gazebo_msgs::ContactsState frontRightLowerLeg;
		gazebo_msgs::ContactsState frontLeftLowerLeg;

		gazebo_msgs::ModelStates modelstates; 

		rosgraph_msgs::Clock unity_time; 
};

#endif

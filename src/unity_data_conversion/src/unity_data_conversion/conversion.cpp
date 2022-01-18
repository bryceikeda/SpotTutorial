#include "unity_data_conversion/conversion.h"

Conversion::Conversion(){
	modelstates.name.push_back("spot1");
	modelstates.pose.push_back(geometry_msgs::Pose());
	modelstates.twist.push_back(geometry_msgs::Twist());
}


void 
Conversion::
handle_gazeboModelStates(const std_msgs::Float64MultiArray& msg){
	modelstates.pose[0].position.x = msg.data[0];
	modelstates.pose[0].position.y = msg.data[1];
	modelstates.pose[0].position.z = msg.data[2];
	modelstates.pose[0].orientation.x = msg.data[3];
	modelstates.pose[0].orientation.y = msg.data[4];
	modelstates.pose[0].orientation.z = msg.data[5];
	modelstates.pose[0].orientation.w = msg.data[6];
}

void 
Conversion::
handle_rearRightLowerLeg(const std_msgs::Float64& msg)
{
	double in = msg.data; 
	if(in > 0){

		rearRightLowerLeg.states.push_back(gazebo_msgs::ContactState());
	}
	else{
		rearRightLowerLeg.states.clear(); 
	}
}


	
void
Conversion::
handle_rearLeftLowerLeg(const std_msgs::Float64& msg)
{
	double in = msg.data;
	if(in > 0){
		rearLeftLowerLeg.states.push_back(gazebo_msgs::ContactState()); 
	}
	else{
		rearLeftLowerLeg.states.clear(); 

	}
}

void 
Conversion::
handle_frontRightLowerLeg(const std_msgs::Float64& msg)
{
	double in = msg.data; 
	if(in > 0){
		frontRightLowerLeg.states.push_back(gazebo_msgs::ContactState());
 

	}
	else{
		frontRightLowerLeg.states.clear(); 

	}
}

void 
Conversion::
handle_frontLeftLowerLeg(const std_msgs::Float64& msg)
{
	double in = msg.data; 
	if(in > 0){
		frontLeftLowerLeg.states.push_back(gazebo_msgs::ContactState());


	}
	else{
		frontLeftLowerLeg.states.clear(); 

	}
}

void 
Conversion::
handle_time(const std_msgs::Float64& msg){
	std_msgs::Float64 data = msg;
       	ros::Time sec(data.data);
	unity_time.clock = sec; 
	//ROS_INFO("%f", data.data);
		
}


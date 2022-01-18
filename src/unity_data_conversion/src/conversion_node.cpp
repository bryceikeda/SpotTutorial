#include "unity_data_conversion/conversion.h"

int main(int argc, char* argv[]){
	Conversion conversion; 
	ros::init(argc, argv, "conversion_node");
	ros::NodeHandle node_handle; 


	ros::Subscriber gazeboModelStates_subscriber = node_handle.subscribe("/gazebo/model_states/floatarray", 1, &Conversion::handle_gazeboModelStates, &conversion);
	ros::Subscriber rearRightLowerLeg_subscriber = node_handle.subscribe("/spot1/rear_right_lower_leg_contactfloat", 1, &Conversion::handle_rearRightLowerLeg, &conversion);
	ros::Subscriber rearLeftLowerLeg_subscriber = node_handle.subscribe("/spot1/rear_left_lower_leg_contactfloat", 1, &Conversion::handle_rearLeftLowerLeg, &conversion);
	ros::Subscriber frontRightLowerLeg_subscriber = node_handle.subscribe("/spot1/front_right_lower_leg_contactfloat", 1, &Conversion::handle_frontRightLowerLeg, &conversion);
	ros::Subscriber frontLeftLowerLeg_subscriber = node_handle.subscribe("/spot1/front_left_lower_leg_contactfloat", 1, &Conversion::handle_frontLeftLowerLeg, &conversion);
	
	ros::Subscriber unitytime_subscriber = node_handle.subscribe("/unity/time", 10, &Conversion::handle_time, &conversion);
	conversion.simtime_publisher = node_handle.advertise<rosgraph_msgs::Clock>("/clock", 1, true);
	
	conversion.modelstates_publisher = node_handle.advertise<gazebo_msgs::ModelStates>("/gazebo/model_states", 1, true);
	
	conversion.rearRightLowerLeg_publisher = node_handle.advertise<gazebo_msgs::ContactsState>("/spot1/rear_right_lower_leg" , 1, true);
	conversion.rearLeftLowerLeg_publisher = node_handle.advertise<gazebo_msgs::ContactsState>("/spot1/rear_left_lower_leg" , 1, true);
	conversion.frontRightLowerLeg_publisher = node_handle.advertise<gazebo_msgs::ContactsState>("/spot1/front_right_lower_leg" , 1, true);
	conversion.frontLeftLowerLeg_publisher = node_handle.advertise<gazebo_msgs::ContactsState>("/spot1/front_left_lower_leg" , 1, true);

	sleep(1);
	double frequency = 60.0; 
	ros::Rate timer(frequency);
	while( ros::ok()){
		conversion.simtime_publisher.publish(conversion.unity_time);
		conversion.modelstates_publisher.publish(conversion.modelstates);
		conversion.rearRightLowerLeg_publisher.publish(conversion.rearRightLowerLeg);
		conversion.rearLeftLowerLeg_publisher.publish(conversion.rearLeftLowerLeg);
		conversion.frontRightLowerLeg_publisher.publish(conversion.frontRightLowerLeg);
		conversion.frontLeftLowerLeg_publisher.publish(conversion.frontLeftLowerLeg);
		ros::spinOnce();
	}

	return 0; 
}

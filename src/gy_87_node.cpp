// ROS libraries
#include "ros/ros.h"

// Sensor libraries
#include "GY87.h"

// Main function
int main(int argc, char **argv)
{
	// Init ROS node
	ROS_INFO("gy_87_node: Init ROS...");
	ros::init(argc, argv, "gy_87");
	ros::NodeHandle n;
	ros::Rate loop_rate(50);

	// Init breakout board
	gy_87::GY87 gy = gy_87::GY87(n);

	ROS_INFO("gy_87_node: Running...");
	while (ros::ok())
	{
		// Read out and publish sensor data
		gy.publish();
	
		// ROS
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}

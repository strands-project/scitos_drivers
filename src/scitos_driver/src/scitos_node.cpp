#include <ros/ros.h>

#include <scitos_driver/ScitosG5.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "scitos_ros");
	mira::Framework framework(argc, argv, true);

	ROS_INFO("Creating G5...");
	ScitosG5 s;
	ROS_INFO("Going into main loop.");
	ros::spin();
	return 0;
}

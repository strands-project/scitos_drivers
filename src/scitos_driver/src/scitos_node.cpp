#include <ros/ros.h>

#include <scitos_driver/ScitosG5.h>

#include <boost/thread.hpp>

int main(int argc, char **argv) {
	mira::Framework framework(argc, argv, true);

	//	printf("waiting for param interface...\n");
	//#	robot_->getMiraAuthority().waitForServiceInterface("IRobotModelProvider");
	//#	printf("done.\n");

	while (!framework.isInExec()) {
	  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	}


	ros::init(argc, argv, "scitos_ros");

	ROS_INFO("Creating G5...");
	ScitosG5 s;
	ROS_INFO("Going into main loop.");
	ros::spin();
	return 0;
}

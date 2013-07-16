#include <ros/ros.h>

#include <scitos_mira/ScitosG5.h>

#include <boost/thread.hpp>
#include <string>
#include <vector>
int main(int argc, char **argv) {
	ros::init(argc, argv, "scitos_node");

	std::string  config_file, port_number;
	std::vector<std::string> args;

	if (argc < 2) { // no arguments, so use ROS parameters.

	  if (ros::param::get("~config_file", config_file))  {
		args.push_back(std::string("-c"));
		args.push_back(config_file);
	  } else {
		ROS_ERROR("Can't read parameter 'config_file'");
		return 1;
	  }
	  if (ros::param::get("~server_port", port_number))  {
		args.push_back(std::string("-p"));
		args.push_back(port_number);
		ROS_INFO_STREAM("Loading with MIRA multiprocess communication support on port " << port_number);
	  } else {
		ROS_INFO("Not loading with MIRA multiprocess support.");
	  }
	} else {
	  for (int i=1; i<argc;i++)
		args.push_back(std::string(argv[i]));
	}
	
	mira::Framework framework(args, true);
	
	while (!framework.isInExec()) {
	  boost::this_thread::sleep(boost::posix_time::milliseconds(50));
	}
	// above is no use.
	boost::this_thread::sleep(boost::posix_time::milliseconds(5000));

	

	ROS_INFO("Creating G5...");
	ScitosG5 s;
	ROS_INFO("Going into main loop.");
	s.spin();
	return 0;
}

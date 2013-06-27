#include <scitos_driver/ScitosG5.h>

#include <scitos_driver/ModuleFactory.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>

ScitosG5::ScitosG5() : authority_("/", "sctios_ros", mira::Authority::ANONYMOUS),
					    node_() {
    ROS_INFO("Creating SCITOS G5 instance.");
	// TODO: Read XML file or similar to figure what modules are present on robot
	//       Maybe lookup on MIRA framework.
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("Drive"), this) );
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("Charger"), this) );
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("EBC"), this) );
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("Display"), this) );
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("Head"), this) );
    initialize();
}

void ScitosG5::initialize() {
  // Initialise all of the modules
    for(std::vector<ScitosModule*>::iterator it = modules_.begin(); it != modules_.end(); ++it) {
    	(*it)->initialize();
    }
}

void ScitosG5::spin() {
  //  ros::spin();
  ros::Rate r(30);
  while (ros::ok()) {
  	ros::spinOnce();
	for (std::vector< boost::function<void ()> >::iterator i = spin_functions_.begin(); i!=spin_functions_.end(); i++){
	  (*i)();
	}
	r.sleep();
  }
}

ScitosG5::~ScitosG5() {
    for(std::vector<ScitosModule*>::iterator it = modules_.begin(); it != modules_.end(); ++it) {
	  delete (*it);
    }
}

mira::Authority& ScitosG5::getMiraAuthority() {
	return authority_;
}

ros::NodeHandle& ScitosG5::getRosNode() {
	return node_;
}

tf::TransformBroadcaster& ScitosG5::getTFBroadcaster() {
	return tf_broadcaster_;
}

void ScitosG5::registerSpinFunction(boost::function<void ()> function) {
    spin_functions_.push_back(function);
}

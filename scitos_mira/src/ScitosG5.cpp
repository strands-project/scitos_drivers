#include <scitos_mira/ScitosG5.h>

#include <scitos_mira/ModuleFactory.h>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <ros/spinner.h>

ScitosG5::ScitosG5(std::vector<std::string> modules) : authority_("/", "scitos_ros", mira::Authority::ANONYMOUS),
					    node_() {
    ROS_INFO("Creating SCITOS G5 instance.");

    ModuleFactory *factory = ModuleFactory::Get();
    for (std::vector<std::string>::iterator i = modules.begin(); i!=modules.end(); i++) {
      ROS_INFO_STREAM("Loading module " << *i);
      if (!factory->CheckForModule(*i)) {
		ROS_ERROR_STREAM("A non existent module was trying to be created. Name=" << *i<<"\n will try to continue without!");
      } else {
		modules_.push_back( factory->CreateModule(*i, this) );
      }
    }

    initialize();
}

void ScitosG5::initialize() {
  // Initialise all of the modules
    for(std::vector<ScitosModule*>::iterator it = modules_.begin(); it != modules_.end(); ++it) {
    	(*it)->initialize();
    }
}

void ScitosG5::spin() {
  ros::AsyncSpinner spinner(1);
  spinner.start();
  ros::Rate r(5);
  while (ros::ok()) {
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

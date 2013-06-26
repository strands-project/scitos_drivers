#include <scitos_driver/ScitosG5.h>

#include <scitos_driver/ModuleFactory.h>


ScitosG5::ScitosG5() : authority_("/", "sctios_ros", mira::Authority::ANONYMOUS),
					    node_() {
    ROS_INFO("Creating SCITOS G5 instance.");
	// TODO: Read XML file or similar to figure what modules are present on robot
	//       Maybe lookup on MIRA framework.
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("Drive"), this) );
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("Charger"), this) );
    modules_.push_back( ModuleFactory::Get()->CreateModule(std::string("EBC"), this) );
    initialize();
}

void ScitosG5::initialize() {
  // Initialise all of the modules
    for(std::vector<ScitosModule*>::iterator it = modules_.begin(); it != modules_.end(); ++it) {
    	(*it)->initialize();
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

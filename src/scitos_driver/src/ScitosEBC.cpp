#include "scitos_driver/ScitosEBC.h"

ScitosEBC::ScitosEBC() : ScitosModule() {
	ROS_INFO("Creating EBC module.");
}

void ScitosEBC::initialize() {
	reconfigure_srv_.setCallback(boost::bind(&ScitosEBC::reconfigure_callback, this, _1, _2));

}

void ScitosEBC::reconfigure_callback( scitos_driver::EBCParametersConfig& config, uint32_t level) {
	ROS_DEBUG("Reconfigure request on ScitosEBC module.");
	//Set the MIRA parameters to what was selected...
	
}


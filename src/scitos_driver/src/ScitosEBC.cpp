#include "scitos_driver/ScitosEBC.h"

#include "scitos_driver/ScitosG5.h"

ScitosEBC::ScitosEBC() : ScitosModule(std::string ("EBC")), reconfigure_srv_(name_) {

}

void ScitosEBC::initialize() {
	reconfigure_srv_.setCallback(boost::bind(&ScitosEBC::reconfigure_callback, this, _1, _2));

}

void ScitosEBC::reconfigure_callback( scitos_driver::EBCParametersConfig& config, uint32_t level) {
	ROS_DEBUG("Reconfigure request on ScitosEBC module.");
	//Set the MIRA parameters to what was selected...
	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), std::string("EBC7.Port1_5V.Enabled"), std::string("true")); 
	r.timedWait(mira::Duration::seconds(1));
	r.get();
	
}


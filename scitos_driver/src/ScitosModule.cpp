#include <scitos_driver/ScitosModule.h>
#include <string>
#include <scitos_driver/ScitosG5.h>
ScitosModule::ScitosModule(std::string name) : name_(name), module_handle_(name_) {
  ROS_INFO("Initialising %s module..",name_.c_str());
}



ScitosModule::~ScitosModule() {

}


bool ScitosModule::set_mira_param_(std::string param_name, std::string value) {
  	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot#builtin", std::string("setProperty"), param_name, value); 
	r.timedWait(mira::Duration::seconds(1)); // timeout after a second of trying...
	r.get(); // throw excepton if no good...
	return true; // TODO: catch exception and return correctly
}

std::string ScitosModule::get_mira_param_(std::string param_name) {
  mira::RPCFuture<std::string> r = robot_->getMiraAuthority().callService<std::string>("/robot/Robot#builtin", std::string("getProperty"), param_name); 
	r.timedWait(mira::Duration::seconds(1)); // timeout after a second of trying...
	return r.get(); // throw excepton if no good...
}


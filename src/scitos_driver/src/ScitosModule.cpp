#include <scitos_driver/ScitosModule.h>
#include <string>

ScitosModule::ScitosModule(std::string name) : name_(name), module_handle_(name_) {
  ROS_INFO("Initialising %s module..",name_.c_str());
}



ScitosModule::~ScitosModule() {

}

#include "scitos_mira/ModuleFactory.h"

#include "scitos_mira/ScitosCharger.h"
#include "scitos_mira/ScitosDisplay.h"
#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosEBC.h"
#include "scitos_mira/ScitosHead.h"

ModuleFactory::ModuleFactory() {
  //	Register("Charger", &ScitosCharger::Create);
  //	/Register("Display", &ScitosDisplay::Create);
  	Register("Drive", &ScitosDrive::Create);
  //	Register("EBC", &ScitosEBC::Create);
  //	Register("Head", &ScitosHead::Create);
}

void ModuleFactory::Register(const std::string &name, ModuleCreator func) {
	modules_[name] = func;
}

ScitosModule *ModuleFactory::CreateModule(std::string name, ScitosG5 *robot) {
  std::map<std::string, ModuleCreator>::iterator it = modules_.find(name);
  if (it != modules_.end()) {
    ScitosModule *mod = it->second();
    mod->setRobot(robot);
    return mod;
  }
  ROS_ERROR("Trying to create unknown Scitos module.");
  return NULL;
}

bool ModuleFactory::CheckForModule(const std::string &name) {
  std::map<std::string, ModuleCreator>::iterator it = modules_.find(name);
  if (it != modules_.end()) {
    return true;
  }
  return false;
}

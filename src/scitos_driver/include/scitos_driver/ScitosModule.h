/* ScitosModule.h
 *      Author: chris burbridge
 *
 * Base class for all Scitos modules (Head, EBC, Drive etc)
 */

#ifndef SCITOSMODULE_H_
#define SCITOSMODULE_H_

#include <ros/ros.h>
#include <fw/Framework.h>
#include <string>

class ScitosG5;

class ScitosModule {
public:
  ScitosModule(std::string name);
	void setRobot(ScitosG5 *robot) { robot_ = robot; };

	virtual void initialize() = 0;

	virtual ~ScitosModule();

protected:
	bool set_mira_param_(std::string param_name, std::string value);
  ScitosG5 *robot_;
  std::string name_;
  ros::NodeHandle module_handle_;

};

typedef ScitosModule* (*ModuleCreator)(void);

#endif /* SCITOSMODULE_H_ */

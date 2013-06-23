/* ScitosModule.h
 *      Author: chris burbridge
 *
 * Base class for all Scitos modules (Head, EBC, Drive etc)
 */

#ifndef SCITOSMODULE_H_
#define SCITOSMODULE_H_

#include <ros/ros.h>
#include <fw/Framework.h>

class ScitosG5;

class ScitosModule {
public:
	ScitosModule();
	void setRobot(ScitosG5 *robot) { robot_ = robot; };

	virtual void initialize() = 0;

	virtual ~ScitosModule();

protected:
  ScitosG5 *robot_;
};

typedef ScitosModule* (*ModuleCreator)(void);

#endif /* SCITOSMODULE_H_ */

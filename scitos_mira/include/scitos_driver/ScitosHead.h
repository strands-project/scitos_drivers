/* ScitosHead.h
 *      Author: chris burbridge
 *
 * Module for interfacing the HRI head module
 * See http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#HeadG5_Section
 */
#ifndef SCITOHEAD_H_
#define SCITOHEAD_H_

#include <scitos_mira/ScitosModule.h>
#include <sensor_msgs/JointState.h>

class ScitosHead: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosHead();
	}

	void initialize();
	void joint_state_command_callback(const sensor_msgs::JointState::ConstPtr& msg);
	void publish_joint_state_actual();
private:
	ScitosHead();
	ros::Subscriber joint_state_command_subscriber_;
	ros::Publisher  joint_state_actual_pub_;
};

#endif /* SCITOSHEAD_H_ */

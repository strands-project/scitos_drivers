#include "scitos_driver/ScitosHead.h"
#include <scitos_driver/ScitosG5.h>

ScitosHead::ScitosHead() : ScitosModule(std::string ("Head")) {

}

void ScitosHead::initialize() {
  joint_state_command_subscriber_ = robot_->getRosNode().subscribe("/head_commanded_state", 10, &ScitosHead::joint_state_command_callback,
				this);
}

void ScitosHead::joint_state_command_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  unsigned int i=0;
  for (i=0; i< msg->name.size(); i++) {
	if (msg->name[i].compare(std::string("LeftEyeLid"))==0 ) {
	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyeLidUpDown",
																			 (unsigned char)1,
																			 (float)msg->position[i]);
 	} else if (msg->name[i].compare(std::string("RightEyeLid"))==0 ) {
	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyeLidUpDown",
																			 (unsigned char)0,
																			 (float)msg->position[i]);
 	} else if (msg->name[i].compare(std::string("HeadPan"))==0 ) {
	  	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveHeadLeftRight",
																			 (float)msg->position[i]);
 	} else if (msg->name[i].compare(std::string("HeadTilt"))==0 ) {
	  	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveHeadUpDown",
																			 (float)msg->position[i]);
 	} else if (msg->name[i].compare(std::string("EyesPan"))==0 ) {
	  	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyesLeftRight",
																			 (float)msg->position[i]);
 	} else if (msg->name[i].compare(std::string("EyesTilt"))==0 ) {
	  	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyesUpDown",
																			 (float)msg->position[i]);
 	} else {
	  ROS_WARN("Trying to control non existing joint in the head.");
	}
  }
}

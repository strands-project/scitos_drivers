#include "scitos_driver/ScitosHead.h"
#include <scitos_driver/ScitosG5.h>

ScitosHead::ScitosHead() : ScitosModule(std::string ("Head")) {

}

void ScitosHead::initialize() {
  joint_state_command_subscriber_ = robot_->getRosNode().subscribe("/head/commanded_state", 10, &ScitosHead::joint_state_command_callback,
				this);
  joint_state_actual_pub_ = robot_->getRosNode().advertise<sensor_msgs::JointState>("/head/actual_state", 20);
  robot_->registerSpinFunction(boost::bind(&ScitosHead::publish_joint_state_actual, this));
}

void ScitosHead::joint_state_command_callback(const sensor_msgs::JointState::ConstPtr& msg) {
  unsigned int i=0;
  for (i=0; i< msg->name.size(); i++) {
	if (msg->name[i].compare(std::string("EyeLidLeft"))==0 ) {
	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyeLidUpDown",
																			 (unsigned char)0,
																			 (float)msg->position[i]);
 	} else if (msg->name[i].compare(std::string("EyeLidRight"))==0 ) {
	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyeLidUpDown",
																			 (unsigned char)1,
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

void ScitosHead::publish_joint_state_actual() {
  sensor_msgs::JointState js;
  std::string value;
  js.name.push_back(std::string("EyeLidLeft"));
  js.name.push_back(std::string("EyeLidRight"));
  js.name.push_back(std::string("HeadPan"));
  js.name.push_back(std::string("HeadTilt"));
  js.name.push_back(std::string("EyesPan"));
  js.name.push_back(std::string("EyesTilt"));

  for (std::vector<std::string>::iterator it = js.name.begin(); it != js.name.end(); it++) {
	value = get_mira_param_(std::string("Head.")+*it);
	js.position.push_back(::atof(value.c_str()) );
  }
  if (joint_state_actual_pub_)
	joint_state_actual_pub_.publish(js);
  
}


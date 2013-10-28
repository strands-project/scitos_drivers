#include "scitos_mira/ScitosHead.h"
#include <scitos_mira/ScitosG5.h>
#include <rpc/RPCError.h>

ScitosHead::ScitosHead() : ScitosModule(std::string ("Head")) {

}

void ScitosHead::initialize() {
  joint_state_command_subscriber_ = robot_->getRosNode().subscribe("/head/commanded_state", 10, &ScitosHead::joint_state_command_callback,
				this);
  headlight_state_command_subscriber_ = robot_->getRosNode().subscribe("/head/cmd_light_state", 10, &ScitosHead::headlight_state_command_callback,
				this);

  joint_state_actual_pub_ = robot_->getRosNode().advertise<sensor_msgs::JointState>("/head/actual_state", 20);
  robot_->registerSpinFunction(boost::bind(&ScitosHead::publish_joint_state_actual, this));
}

void ScitosHead::headlight_state_command_callback(const scitos_msgs::HeadLightState::ConstPtr& msg) {
  //todo: Contact ML to get some documentation on how the head lights work.
  set_mira_param_("Head.HeadLightInterval", std::to_string(msg->HeadLightInterval) );

  set_mira_param_("Head.LEDState0", std::to_string(msg->LEDState0) );
  set_mira_param_("Head.LEDPhase0", std::to_string(msg->LEDPhase0) );
  set_mira_param_("Head.LEDAmplitude0", std::to_string(msg->LEDAmplitude0) );

  set_mira_param_("Head.LEDState1", std::to_string(msg->LEDState1) );
  set_mira_param_("Head.LEDPhase1", std::to_string(msg->LEDPhase1) );
  set_mira_param_("Head.LEDAmplitude1", std::to_string(msg->LEDAmplitude1) );

  set_mira_param_("Head.LEDState2", std::to_string(msg->LEDState2) );
  set_mira_param_("Head.LEDPhase2", std::to_string(msg->LEDPhase2) );
  set_mira_param_("Head.LEDAmplitude2", std::to_string(msg->LEDAmplitude2) );

  set_mira_param_("Head.LEDState3", std::to_string(msg->LEDState3) );
  set_mira_param_("Head.LEDPhase3", std::to_string(msg->LEDPhase3) );
  set_mira_param_("Head.LEDAmplitude3", std::to_string(msg->LEDAmplitude3) );

  set_mira_param_("Head.LEDState4", std::to_string(msg->LEDState4) );
  set_mira_param_("Head.LEDPhase4", std::to_string(msg->LEDPhase4) );
  set_mira_param_("Head.LEDAmplitude4", std::to_string(msg->LEDAmplitude4) );

  set_mira_param_("Head.LEDState5", std::to_string(msg->LEDState5) );
  set_mira_param_("Head.LEDPhase5", std::to_string(msg->LEDPhase5) );
  set_mira_param_("Head.LEDAmplitude5", std::to_string(msg->LEDAmplitude5) );

  set_mira_param_("Head.LEDState6", std::to_string(msg->LEDState6) );
  set_mira_param_("Head.LEDPhase6", std::to_string(msg->LEDPhase6) );
  set_mira_param_("Head.LEDAmplitude6", std::to_string(msg->LEDAmplitude6) );

  set_mira_param_("Head.LEDState7", std::to_string(msg->LEDState7) );
  set_mira_param_("Head.LEDPhase7", std::to_string(msg->LEDPhase7) );
  set_mira_param_("Head.LEDAmplitude7", std::to_string(msg->LEDAmplitude7) );
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
 	} else if (msg->name[i].compare(std::string("EyeLids"))==0 ) {
	  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
																			 "moveEyeLidUpDown",
																			 (unsigned char)2,
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
  try {
    for (std::vector<std::string>::iterator it = js.name.begin(); it != js.name.end(); it++) {
	    value = get_mira_param_(std::string("Head.")+*it);
	    js.position.push_back(::atof(value.c_str()) );
    }
    if (joint_state_actual_pub_)
	    joint_state_actual_pub_.publish(js);
  } catch (mira::XRPC& e) {
    ROS_WARN("Missing head angle publication as MIRA parameter error.");
  }
  
}


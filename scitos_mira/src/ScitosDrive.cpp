
#include "scitos_mira/ScitosDrive.h"
#include "scitos_mira/ScitosG5.h"

#include <transform/RigidTransform.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt64.h>

uint64 MAGNETIC_BARRIER_RFID_CODE=0xabababab;

ScitosDrive::ScitosDrive() : ScitosModule(std::string ("Drive")) { 
}

void ScitosDrive::initialize() {
  odometry_pub_ = robot_->getRosNode().advertise<nav_msgs::Odometry>("/odom", 20);
  bumper_pub_ = robot_->getRosNode().advertise<std_msgs::Bool>("/bumper", 20);
  mileage_pub_ = robot_->getRosNode().advertise<std_msgs::Float32>("/mileage", 20);
  motorstatus_pub_ = robot_->getRosNode().advertise<scitos_msgs::MotorStatus>("/motor_status", 20);
  rfid_pub_ = robot_->getRosNode().advertise<std_msgs::UInt64>("/rfid", 20);
  magnetic_barrier_pub_ = robot_->getRosNode().advertise<scitos_msgs::BarrierStatus>("/barrier_status", 20);
  emergency_stop_pub_ = robot_->getRosNode().advertise<std_msgs::Bool>("/emergency_stop_status", 20, true);
  
  robot_->getMiraAuthority().subscribe<mira::robot::Odometry2>("/robot/Odometry", //&ScitosBase::odometry_cb);
							       &ScitosDrive::odometry_data_callback, this);
  robot_->getMiraAuthority().subscribe<bool>("/robot/Bumper",
					     &ScitosDrive::bumper_data_callback, this);
  robot_->getMiraAuthority().subscribe<float>("/robot/Mileage",
					      &ScitosDrive::mileage_data_callback, this);
  robot_->getMiraAuthority().subscribe<uint8>("/robot/MotorStatus",
					      &ScitosDrive::motor_status_callback, this);
  robot_->getMiraAuthority().subscribe<uint64>("/robot/RFIDFloorTag",
					      &ScitosDrive::rfid_status_callback, this);
	
  cmd_vel_subscriber_ = robot_->getRosNode().subscribe("/cmd_vel", 1000, &ScitosDrive::velocity_command_callback,
						       this);
  
  reset_motor_stop_service_ = robot_->getRosNode().advertiseService("/reset_motorstop", &ScitosDrive::reset_motor_stop, this);
  reset_odometry_service_ = robot_->getRosNode().advertiseService("/reset_odometry", &ScitosDrive::reset_odometry, this);
  emergency_stop_service_ = robot_->getRosNode().advertiseService("/emergency_stop", &ScitosDrive::emergency_stop, this);
  enable_motors_service_ = robot_->getRosNode().advertiseService("/enable_motors", &ScitosDrive::enable_motors, this);
  change_force_service_ = robot_->getRosNode().advertiseService("/change_force", &ScitosDrive::change_force, this);
  enable_rfid_service_ = robot_->getRosNode().advertiseService("/enable_rfid", &ScitosDrive::enable_rfid, this);
  reset_barrier_stop_service_ = robot_->getRosNode().advertiseService("/reset_barrier_stop", &ScitosDrive::reset_barrier_stop, this);

  bool magnetic_barrier_enabled = true;
  ros::param::param("~magnetic_barrier_enabled", magnetic_barrier_enabled, magnetic_barrier_enabled);
  if (magnetic_barrier_enabled) {
    set_mira_param_("MainControlUnit.RearLaser.Enabled", "true");
  }  else {
    ROS_WARN("Magnetic barrier motor stop not enabled.");
    set_mira_param_("MainControlUnit.RearLaser.Enabled", "false");
  }

  emergency_stop_.data = false;
  barrier_status_.barrier_stopped = false;
  barrier_status_.last_detection_stamp = ros::Time(0);
  robot_->registerSpinFunction(boost::bind(&ScitosDrive::publish_barrier_status, this));

}

void ScitosDrive::velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg) {
  if ( !barrier_status_.barrier_stopped && !emergency_stop_.data) {
      mira::RigidTransform<float, 2> speed(msg->linear.x, 0, msg->angular.z);
      mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
									     "setVelocity", speed);
      r.wait();
  }
}

void ScitosDrive::bumper_data_callback(mira::ChannelRead<bool> data) {
	std_msgs::Bool out;
	out.data=data->value();					 
	bumper_pub_.publish(out);
}

void ScitosDrive::mileage_data_callback(mira::ChannelRead<float> data) {
  std_msgs::Float32 out;
  out.data = data->value();					   
  mileage_pub_.publish(out);
}

void ScitosDrive::rfid_status_callback(mira::ChannelRead<uint64> data) {
  if (data->value() == MAGNETIC_BARRIER_RFID_CODE) {
    barrier_status_.barrier_stopped = true;
    barrier_status_.last_detection_stamp = ros::Time().fromNSec(data->timestamp.toUnixNS()); //Before it was ros::Time::now(). Changed it to the actual mira timestamp 
  }
  std_msgs::UInt64 out;
  out.data = data->value();
  rfid_pub_.publish(out);
}


void ScitosDrive::motor_status_callback(mira::ChannelRead<uint8> data) {
  ros::Time time_now = ros::Time().fromNSec(data->timestamp.toUnixNS()); //Before it was ros::Time::now(). Changed it to the actual mira timestamp
  
  scitos_msgs::MotorStatus s;
  s.header.stamp=time_now;
  s.normal = (*data) & 1;
  s.motor_stopped = (*data) & (1 << 1);
  s.free_run = (*data) & (1 << 2);
  s.emergency_button_pressed = (*data) & (1 << 3);
  s.bumper_pressed = (*data) & (1 << 4);
  s.bus_error = (*data) & (1 << 5);
  s.stall_mode_flag = (*data) & (1 << 6);
  s.internal_error_flag = (*data) & (1 << 7);
  
  motorstatus_pub_.publish(s);
}

void ScitosDrive::publish_barrier_status() {
  barrier_status_.header.stamp = ros::Time::now();
  magnetic_barrier_pub_.publish(barrier_status_);
}

void ScitosDrive::odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data ) {
	/// new odometry data through mira; put it out in ros
	ros::Time odom_time = ros::Time().fromNSec(data->timestamp.toUnixNS()); //Before it was ros::Time::now(). Changed it to the actual mira timestamp
	geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(
			data->value().pose.phi());

	// Publish as a nav_msgs::Odometry
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = odom_time;
	odom_msg.header.frame_id = "/odom";
	odom_msg.child_frame_id = "/base_footprint";

	// set the position
	odom_msg.pose.pose.position.x = data->value().pose.x();
	odom_msg.pose.pose.position.y = data->value().pose.y();
	odom_msg.pose.pose.orientation = orientation;

	// set the velocity
	odom_msg.twist.twist.linear.x = data->value().velocity.x();
	odom_msg.twist.twist.angular.z = data->value().velocity.phi();

	odometry_pub_.publish(odom_msg);

	// Publish a TF
	geometry_msgs::TransformStamped odom_tf;
	odom_tf.header.stamp = odom_time;
	odom_tf.header.frame_id = "/odom";
	odom_tf.child_frame_id = "/base_footprint";

	odom_tf.transform.translation.x = data->value().pose.x();
	odom_tf.transform.translation.y = data->value().pose.y();
	odom_tf.transform.translation.z = 0.0;
	odom_tf.transform.rotation = orientation;
	// send the transform
	robot_->getTFBroadcaster().sendTransform(odom_tf);
}

bool ScitosDrive::reset_motor_stop(scitos_msgs::ResetMotorStop::Request  &req, scitos_msgs::ResetMotorStop::Response &res) {
  //  call_mira_service
  emergency_stop_.data = false;
  emergency_stop_pub_.publish(emergency_stop_);
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("resetMotorStop"));
  r.timedWait(mira::Duration::seconds(1));
  r.get(); 

  return true;
}

bool ScitosDrive::reset_odometry(scitos_msgs::ResetOdometry::Request  &req, scitos_msgs::ResetOdometry::Response &res) {
  //  call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("resetOdometry"));
  r.timedWait(mira::Duration::seconds(1));
  r.get(); 

  return true;
}


bool ScitosDrive::emergency_stop(scitos_msgs::EmergencyStop::Request  &req, scitos_msgs::EmergencyStop::Response &res) {
  //  call_mira_service
  emergency_stop_.data = true;
  emergency_stop_pub_.publish(emergency_stop_);
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("emergencyStop"));
  r.timedWait(mira::Duration::seconds(1));
  r.get(); 

  return true;
}


bool ScitosDrive::enable_motors(scitos_msgs::EnableMotors::Request  &req, scitos_msgs::EnableMotors::Response &res) {
  //  call_mira_service
  mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot", std::string("enableMotors"),(bool)req.enable);
  r.timedWait(mira::Duration::seconds(1));
  r.get(); 

  return true;
}

bool ScitosDrive::change_force(scitos_msgs::ChangeForce::Request  &req, scitos_msgs::ChangeForce::Response &res) {
	// change mira params 
	return set_mira_param_("MainControlUnit.Force",mira::toString(req.force));
}

bool ScitosDrive::enable_rfid(scitos_msgs::EnableRfid::Request  &req, scitos_msgs::EnableRfid::Response &res) {
  if (req.enable == true) 
    return set_mira_param_("MainControlUnit.RearLaser.Enabled","true");
  if (req.enable == false) 
    return set_mira_param_("MainControlUnit.RearLaser.Enabled","false");
  return false;
}

bool ScitosDrive::reset_barrier_stop(scitos_msgs::ResetBarrierStop::Request  &req, scitos_msgs::ResetBarrierStop::Response &res) {
  barrier_status_.barrier_stopped = false;
  return true;
}

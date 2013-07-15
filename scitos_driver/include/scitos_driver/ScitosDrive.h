/* ScitosDrive.h
 *      Author: chris burbridge
 *
 * Module for interfacing all related to drive: odometry, motor controller state etc etc
 * http://www.mira-project.org/MIRA-doc/domains/robot/SCITOS/index.html#Drive_Section
 */

#ifndef SCITOSBASE_H
#define SCITOSBASE_H

//#include <ros/ros.h>
//#include <fw/Framework.h>

#include <geometry_msgs/Twist.h>
#include <robot/Odometry.h> //# MIRA odometry
#include <scitos_driver/ScitosModule.h>
#include <scitos_msgs/ResetMotorStop.h>
#include <scitos_msgs/ResetOdometry.h>
#include <scitos_msgs/EmergencyStop.h>
#include <scitos_msgs/EnableMotors.h>

class ScitosDrive: public ScitosModule {
public:
	static ScitosModule*  Create() {
		return new ScitosDrive();
	}

	void initialize();

	void velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg);

	void odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data,	int i);

	void bumper_data_callback(mira::ChannelRead<bool> data, int i);
	void mileage_data_callback(mira::ChannelRead<float> data, int i);

	bool reset_motor_stop(scitos_msgs::ResetMotorStop::Request  &req, scitos_msgs::ResetMotorStop::Response &res);
	bool reset_odometry(scitos_msgs::ResetOdometry::Request  &req, scitos_msgs::ResetOdometry::Response &res);
	bool emergency_stop(scitos_msgs::EmergencyStop::Request  &req, scitos_msgs::EmergencyStop::Response &res);
	bool enable_motors(scitos_msgs::EnableMotors::Request  &req, scitos_msgs::EnableMotors::Response &res);

private:
	ScitosDrive();
	ros::Subscriber cmd_vel_subscriber_;
	ros::Publisher odometry_pub_;
	ros::Publisher bumper_pub_;
	ros::Publisher mileage_pub_;
	ros::ServiceServer reset_motor_stop_service_;
	ros::ServiceServer reset_odometry_service_;
	ros::ServiceServer emergency_stop_service_;
	ros::ServiceServer enable_motors_service_;
};

#endif


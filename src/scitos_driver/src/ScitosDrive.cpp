#include <scitos_driver/ScitosDrive.h>

#include <transform/RigidTransform.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Odometry.h>

#include <tf/transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>

#include <scitos_driver/ScitosG5.h>

ScitosDrive::ScitosDrive() : ScitosModule() { //ScitosG5 *robot, ros::NodeHandle &nh) : ScitosModule(robot, nh){
	ROS_INFO("Creating Drive module.");
}

void ScitosDrive::initialize() {
	robot_->getMiraAuthority().subscribe<mira::robot::Odometry2>("/robot/Odometry", //&ScitosBase::odometry_cb);
			boost::bind(&ScitosDrive::odometry_data_callback, this, _1, 1));
	cmd_vel_subscriber = robot_->getRosNode().subscribe("/cmd_vel", 1000, &ScitosDrive::velocity_command_callback,
				this);
	odometry_pub = robot_->getRosNode().advertise<nav_msgs::Odometry>("/odom", 20);
}

void ScitosDrive::velocity_command_callback(const geometry_msgs::Twist::ConstPtr& msg) {
	mira::RigidTransform<float, 2> speed(msg->linear.x, 0, msg->angular.z);
	mira::RPCFuture<void> r = robot_->getMiraAuthority().callService<void>("/robot/Robot",
			"setVelocity", speed);
	r.wait();
}

void ScitosDrive::odometry_data_callback(mira::ChannelRead<mira::robot::Odometry2> data,
		int i) {
	/// new odometry data through mira; put it out in ros
	ros::Time odom_time = ros::Time::now(); // must be something better?
	geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(
			data->value().pose.phi());

	// Publish as a nav_msgs::Odometry
	nav_msgs::Odometry odom_msg;
	odom_msg.header.stamp = odom_time;
	odom_msg.header.frame_id = "/odom";
	odom_msg.child_frame_id = "/base_link";

	// set the position
	odom_msg.pose.pose.position.x = data->value().pose.x();
	odom_msg.pose.pose.position.y = data->value().pose.y();
	odom_msg.pose.pose.orientation = orientation;

	// set the velocity
	odom_msg.twist.twist.linear.x = data->value().velocity.x();
	odom_msg.twist.twist.angular.z = data->value().velocity.phi();

	odometry_pub.publish(odom_msg);

	// Publish a TF
	geometry_msgs::TransformStamped odom_tf;
	odom_tf.header.stamp = odom_time;
	odom_tf.header.frame_id = "/odom";
	odom_tf.child_frame_id = "/base_link";

	odom_tf.transform.translation.x = data->value().pose.x();
	odom_tf.transform.translation.y = data->value().pose.y();
	odom_tf.transform.translation.z = 0.0;
	odom_tf.transform.rotation = orientation;
	// send the transform
	robot_->getTFBroadcaster().sendTransform(odom_tf);
}

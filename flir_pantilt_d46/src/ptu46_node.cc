#include <string>
#include <ros/ros.h>
#include <ptu46/ptu46_driver.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <std_srvs/Empty.h>

namespace PTU46 {

/**
 * PTU46 ROS Package
 * Copyright (C) 2009 Erik Karulf (erik@cse.wustl.edu)
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
class PTU46_Node {
    public:
        PTU46_Node(ros::NodeHandle& node_handle);
        ~PTU46_Node();

        // Service Control
        void Connect();
        bool ok() {
            return m_pantilt != NULL;
        }
        void Disconnect();

        // Service Execution
        void spinOnce();

        // Callback Methods
        void SetGoalPosition(const sensor_msgs::JointState::ConstPtr& msg);
        void SetGoalVelocity(const sensor_msgs::JointState::ConstPtr& msg);

        void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat);
        bool ResetPtuSrv(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

    protected:
        diagnostic_updater::Updater* m_updater;
        PTU46* m_pantilt;
        ros::NodeHandle m_node;
        ros::Publisher  m_joint_pub;
        ros::Subscriber m_joint_sub;
        ros::ServiceServer m_reset_srv;
        ros::Subscriber m_joint_sub_vel;  
        std::string m_pan_joint_name;
        std::string m_tilt_joint_name;
        bool m_check_limits;
        bool m_velocity_control;

};

PTU46_Node::PTU46_Node(ros::NodeHandle& node_handle)
  :m_pantilt(NULL), m_node(node_handle), m_velocity_control(false) {
    m_updater = new diagnostic_updater::Updater();
    m_updater->setHardwareID("none"); 
    m_updater->add("PTU Status", this, &PTU46_Node::produce_diagnostics);

	// Get the desired joint names
	m_node.param<std::string>("pan_joint_name", m_pan_joint_name, std::string("pan"));
	m_node.param<std::string>("tilt_joint_name", m_tilt_joint_name, std::string("tilt"));
    m_node.param<bool>("check_limits", m_check_limits, true);

	// Set the values to other nodes can get it even if default used
	m_node.setParam("pan_joint_name", m_pan_joint_name);
    m_node.setParam("tilt_joint_name", m_tilt_joint_name);
    m_node.setParam("check_limits", m_check_limits);
}

PTU46_Node::~PTU46_Node() {
    Disconnect();
    delete m_updater;
}

/** Opens the connection to the PTU and sets appropriate parameters.
    Also manages subscriptions/publishers */
void PTU46_Node::Connect() {
    // If we are reconnecting, first make sure to disconnect
    if (ok()) {
        Disconnect();
    }

    // Query for serial configuration
    std::string port;
    m_node.param<std::string>("port", port, PTU46_DEFAULT_PORT);
    int baud;
    m_node.param("baud", baud, PTU46_DEFAULT_BAUD);

    // Connect to the PTU
    ROS_INFO("Attempting to connect to %s...", port.c_str());
    m_pantilt = new PTU46(port.c_str(), baud);
    ROS_ASSERT(m_pantilt != NULL);
    if (! m_pantilt->isOpen()) {
        ROS_ERROR("Could not connect to pan/tilt unit [%s]", port.c_str());
        Disconnect();
        return;
    }
    ROS_INFO("Connected!");

    m_node.setParam("min_tilt", m_pantilt->GetMin(PTU46_TILT));
    m_node.setParam("max_tilt", m_pantilt->GetMax(PTU46_TILT));
    m_node.setParam("min_tilt_speed", m_pantilt->GetMinSpeed(PTU46_TILT));
    m_node.setParam("max_tilt_speed", m_pantilt->GetMaxSpeed(PTU46_TILT));
    m_node.setParam("tilt_step", m_pantilt->GetResolution(PTU46_TILT));

    m_node.setParam("min_pan", m_pantilt->GetMin(PTU46_PAN));
    m_node.setParam("max_pan", m_pantilt->GetMax(PTU46_PAN));
    m_node.setParam("min_pan_speed", m_pantilt->GetMinSpeed(PTU46_PAN));
    m_node.setParam("max_pan_speed", m_pantilt->GetMaxSpeed(PTU46_PAN));
    m_node.setParam("pan_step", m_pantilt->GetResolution(PTU46_PAN));

    // set check limits
    ROS_INFO("check limits:  %d...", m_check_limits);
    m_pantilt->SetCheckLimits(m_check_limits);

	// set velocity control
	if (m_velocity_control) {
	  m_pantilt->SetMode(PTU46_VELOCITY);
	  ROS_INFO("PTU starting in VELOCITY control mode.");
    } else {
	  ROS_INFO("PTU starting in POSITION control mode.");
	  m_pantilt->SetMode(PTU46_POSITION);
	}

    // Publishers : Only publish the most recent reading
    m_joint_pub = m_node.advertise
                  <sensor_msgs::JointState>("state", 1);

    // Subscribers : Only subscribe to the most recent instructions
    m_joint_sub = m_node.subscribe
                  <sensor_msgs::JointState>("cmd", 1, &PTU46_Node::SetGoalPosition, this);
    m_joint_sub_vel = m_node.subscribe
	              <sensor_msgs::JointState>("cmd_vel", 1, &PTU46_Node::SetGoalVelocity, this);

    // Reset service handle
	m_reset_srv = m_node.advertiseService("reset", &PTU46_Node::ResetPtuSrv, this);
}

/** Disconnect */
void PTU46_Node::Disconnect() {
    if (m_pantilt != NULL) {
        delete m_pantilt;   // Closes the connection
        m_pantilt = NULL;   // Marks the service as disconnected
    }
}

/* Service handle for reseting the PTU */
bool PTU46_Node::ResetPtuSrv(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res) {
  m_pantilt->Reset();
  return true;
}

/** Callback for getting new Goal JointState */
void PTU46_Node::SetGoalPosition(const sensor_msgs::JointState::ConstPtr& msg) {
    if (! ok())
        return;
	if (m_velocity_control) {
	  ROS_INFO("Reverting PTU control mode to POSITION");
	  bool success = m_pantilt->SetMode(PTU46_POSITION);
	  if (! success) {
		ROS_WARN("Position goal ignored - error setting position control mode.");
		return;
	  }
	  m_velocity_control = false;
	}
	  
	unsigned int i=0;
	double pan=0;
	double tilt=0;
	double panspeed=0;
	double tiltspeed=0;
	
	for (i=0; i< msg->name.size(); i++) {
	  if (msg->name[i].compare(m_pan_joint_name)==0 ) {
		pan = msg->position[i];
		panspeed = msg->velocity[i];

		m_pantilt->SetPosition(PTU46_PAN, pan);
		m_pantilt->SetSpeed(PTU46_PAN, panspeed);
		
	  } else if (msg->name[i].compare(m_tilt_joint_name)==0 ) {
		tilt = msg->position[i];
		tiltspeed = msg->velocity[i];

		m_pantilt->SetPosition(PTU46_TILT, tilt);
		m_pantilt->SetSpeed(PTU46_TILT, tiltspeed);
		
	  }  else {
		ROS_WARN_STREAM("Trying to control the PTU with a bad joint name. Joint=" << msg->name[i]);
	  }
	}
	
}

/** Callback for getting new velocity Goal JointState */
void PTU46_Node::SetGoalVelocity(const sensor_msgs::JointState::ConstPtr& msg) {
    if (! ok())
        return;
	if (! m_velocity_control) {
	  ROS_INFO("Setting PTU control mode to VELOCITY.");
	  bool success = m_pantilt->SetMode(PTU46_VELOCITY);
	  if (! success) {
		ROS_WARN("Velocity goal ignored - error setting velocity control mode.");
		return;
	  }
	  m_velocity_control = true;
	}

	unsigned int i=0;
	double pan=0;
	double tilt=0;
	double panspeed=0;
	double tiltspeed=0;
	
	for (i=0; i< msg->name.size(); i++) {
	  if (msg->name[i].compare(m_pan_joint_name)==0 ) {
		panspeed = msg->velocity[i];

		m_pantilt->SetSpeed(PTU46_PAN, panspeed);
		
	  } else if (msg->name[i].compare(m_tilt_joint_name)==0 ) {
		tiltspeed = msg->velocity[i];

		m_pantilt->SetSpeed(PTU46_TILT, tiltspeed);
		
	  }  else {
		ROS_WARN_STREAM("Trying to control the PTU with a bad joint name. Joint=" << msg->name[i]);
	  }
	}
	
}

void PTU46_Node::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper &stat) {
     stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "All normal.");
     stat.add("PTU Mode", m_pantilt->GetMode()==PTU46_POSITION ? "Position" : "Velocity" );
}


/**
 * Publishes a joint_state message with position and speed.
 * Also sends out updated TF info.
 */
void PTU46_Node::spinOnce() {
    if (! ok())
        return;

    // Read Position & Speed
    double pan  = m_pantilt->GetPosition(PTU46_PAN);
    double tilt = m_pantilt->GetPosition(PTU46_TILT);

    double panspeed  = m_pantilt->GetSpeed(PTU46_PAN);
    double tiltspeed = m_pantilt->GetSpeed(PTU46_TILT);

    // Publish Position & Speed
    sensor_msgs::JointState joint_state;
    joint_state.header.stamp = ros::Time::now();
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = m_pan_joint_name;
    joint_state.position[0] = pan;
    joint_state.velocity[0] = panspeed;
    joint_state.name[1] = m_tilt_joint_name;
    joint_state.position[1] = tilt;
    joint_state.velocity[1] = tiltspeed;
    m_joint_pub.publish(joint_state);

    m_updater->update();

}

} // PTU46 namespace

int main(int argc, char** argv) {
    ros::init(argc, argv, "ptu");
    ros::NodeHandle n("~");

    // Connect to PTU
    PTU46::PTU46_Node ptu_node = PTU46::PTU46_Node(n);
    ptu_node.Connect();
    if (! ptu_node.ok())
        return -1;

    // Query for polling frequency
    int hz;
    n.param("hz", hz, PTU46_DEFAULT_HZ);
    ros::Rate loop_rate(hz);

    while (ros::ok() && ptu_node.ok()) {
        // Publish position & velocity
        ptu_node.spinOnce();

        // Process a round of subscription messages
        ros::spinOnce();

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }

    if (! ptu_node.ok()) {
        ROS_ERROR("pan/tilt unit disconncted prematurely");
        return -1;
    }

    return 0;
}

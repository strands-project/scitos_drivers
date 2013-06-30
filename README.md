scitos_mira
===========

Scitos G5 drivers that interface ROS to MIRA.

Installation
------------
Install ROS Groovy on the robot: follow http://www.ros.org/wiki/groovy/Installation/Ubuntu

Check out the repository onto the robot and compile:

```
cd ~
git clone https://github.com/strands-project/scitos_mira.git
cd scitos_mira
catkin_make
. devel/setup.bash
```

Next download and build the SICK S300 driver from AIS-Bonn:

```
cd ~
mkdir rosbuild_ws
cd rosbuild_ws
svn co http://ais-bonn-ros-pkg.googlecode.com/svn/trunk/stacks/ais_bonn_drivers/sicks300
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:/localhome/demo/rosbuild_ws
rosmake sicks300 sicks300
```

Make sure that you have the OpenNI drivers installed for the Xtion:

```
sudo apt-get install ros-groovy-openni-camera
sudo apt-get install ros-groovy-openni-launch
```

Edit the OpenNI driver file /etc/openni/GlobalDefaults.ini, find the line "UsbInterface" and uncomment it to force it to use 'BULK' endpoints. It should read  "UsbInterface=2".


Next launch the bridge:

```
roslaunch scitos_launch scitos_mira.launch
```

This will start the Laser, Camera and Scitos base.


Using the robot
---------------

Various aspects of the robot are exposed as ROS services, published topics, subscribed topics and dynamic_reconfigure parameters. These are divided into 5 "modules":
* Drive - for the control of the motors.
* EBC - for controlling the power for extra devices (pan tilt and cameras).
* Head - for controlling the HRI head.
* Display - for the small status display on the base.
* Charger - for battery monitoring and charging control.

### Drive
#### Published topics
* `/odom (nav_msgs::Odometry)`
The odometric position of the robot (Odometry.pose), and it's linear/angular velocity (Odometry.twist.linear.x, Odometry.twist.angular.z).
* `/bumper (std_msgs::Bool)`
State of the robots bumper, published regularly not only when the state changes.
* `/mileage (std_msgs::Float32)`
The distance in metres that the robot has travelled since the beginning of time.

#### Subscribed topics
*  `/cmd_vel (geometry_msgs::Twist)`
Any velocity published on this topic will be sent to the robot motor controller. Twist.linear.x corresponds to the desired linear velocity; Twist.angular.z corresponds to the angular velocity.


#### Services
#### Reconfigure parameters


### Head
#### Published topics
#### Subscribed topics
* `/head/commanded_state (sensor_msgs::JointState)`
To control the robot's head position. There are 6 axis that can be controlled by placing the joint name in JointState.name and the desired state in JointState.position. The axis are:
  * `HeadPan` - the pan joint of the head; 0 to 360 degrees, with a block point at 90 degrees.
  * `HeadTilt` - the tilt of the head; 
  * `EyePan` - the pan of the eyes, without moving the head.
  * `EyeTilt` - the tilt of the eyes, without moving the head.
  * `EyeLidLeft` - the state of the left eye lid, 0..100, 100  fully closed.
  * `EyeLidRight` - the state of the right eye lid, 0..100, 100  fully closed.

#### Services
#### Reconfigure parameters

### EBC
#### Published topics
#### Subscribed topics
#### Services
#### Reconfigure parameters

### Display
#### Published topics
#### Subscribed topics
#### Services
#### Reconfigure parameters

### Charger
#### Published topics
#### Subscribed topics
#### Services
#### Reconfigure parameters

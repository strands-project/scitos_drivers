/* ScitosG5.h
 *      Author: chris burbridge
 *
 * Main class for the robot. Instantiates modules present on the robot, manages
 * MIRA and ROS handles.
 * 
 */

#ifndef SCITOSG5_H
#define SCITOSG5_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <fw/Framework.h>
#include <scitos_mira/ScitosModule.h>
#include <vector>

class ScitosG5{
public:
  ScitosG5();
  ~ScitosG5();

  mira::Authority& getMiraAuthority();
  ros::NodeHandle& getRosNode();

  void initialize();
  void spin();

  void registerSpinFunction(boost::function<void ()> function);
  tf::TransformBroadcaster& getTFBroadcaster();

private:
  mira::Authority authority_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::NodeHandle node_;
  std::vector<ScitosModule*> modules_;
  std::vector< boost::function<void ()> > spin_functions_;

};

#endif

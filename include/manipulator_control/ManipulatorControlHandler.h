#ifndef MANIPULATOR_CONTROL_HANDLER_H
#define MANIPULATOR_CONTROL_HANDLER_H

#include "ElbowHandler.h"
#include "ShoulderHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>

class ManipulatorControlHandler {
private:
  ElbowHandler elbow;
  ShoulderHandler shoulder;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
  void callback_manipulator(const std_msgs::Int64 &msg);
  void setup();
};

#endif // MANIPULATOR_CONTROL_HANDLER_H

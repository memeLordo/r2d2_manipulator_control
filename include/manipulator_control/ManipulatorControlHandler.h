#ifndef MANIPULATOR_CONTROL_HANDLER_H
#define MANIPULATOR_CONTROL_HANDLER_H

#include "ElbowHandler.h"
#include "ShoulderHandler.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <ros/ros.h>

class ManipulatorControlHandler {
private:
  ElbowHandler elbow;
  ShoulderHandler shoulder;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
  void callback_manipulator(const r2d2_msg_pkg::DriverStateConstPtr &msg);
  void setup();
};

#endif // MANIPULATOR_CONTROL_HANDLER_H

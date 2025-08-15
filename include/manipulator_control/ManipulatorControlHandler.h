#ifndef MANIPULATOR_CONTROL_HANDLER_H
#define MANIPULATOR_CONTROL_HANDLER_H

#include "ElbowHandler.h"
#include "PayloadHandler.h"
#include "ShoulderHandler.h"
#include <ros/ros.h>

class ManipulatorControlHandler {
private:
  ElbowHandler elbow;
  ShoulderHandler shoulder;
  PayloadHandler payload;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
  void setup();
  void callback_manipulator();
};

#endif // MANIPULATOR_CONTROL_HANDLER_H

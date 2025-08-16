#ifndef MANIPULATOR_CONTROL_HANDLER_H
#define MANIPULATOR_CONTROL_HANDLER_H

#include "ElbowHandler.h"
#include "PayloadHandler.h"
#include "ShoulderHandler.h"
#include <ros/ros.h>

class ManipulatorControlHandler {
private:
  struct manipulator_t {
    int16_t force_needed{};
    double r0{};
  } params;
  manipulator_t callback_params;

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

#ifndef MANIPULATOR_CONTROL_HANDLER_H
#define MANIPULATOR_CONTROL_HANDLER_H

#include "ElbowHandler.h"
#include "ShoulderHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

class ManipulatorControlHandler {
private:
  ElbowHandler elbowHandler;
  ShoulderHandler shoulderHandler;
  ros::Publisher manipulator_publisher;
  ros::Subscriber manipulator_subscriber;
  ros::ServiceServer test_service;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
  void callback_shoulder(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
};

#endif // MANIPULATOR_CONTROL_HANDLER_H

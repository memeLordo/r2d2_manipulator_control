#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

class ShoulderHandler {
private:
  struct shoulder_t {
  } params;
  ros::Publisher shoulder_publisher;
  ros::Subscriber shoulder_subscriber;
  ros::ServiceServer test_service;
  static const double coeffs[];

  PipeHandler pipeHandler;

public:
  ShoulderHandler(ros::NodeHandle *node);
  void callback_shoulder(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
};

#endif // SHOULDER_HANDLER_H

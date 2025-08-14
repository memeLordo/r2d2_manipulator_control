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
  static const double coeffs[];

  PipeHandler pipeHandler;

  ros::Publisher publisher;
  ros::Subscriber subscriber;
  ros::ServiceServer test_service;

public:
  ShoulderHandler(ros::NodeHandle *node);
  void callback_shoulder(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
  double get_angle();
};

#endif // SHOULDER_HANDLER_H

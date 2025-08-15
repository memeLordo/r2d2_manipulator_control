#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include <cstdint>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

class ElbowHandler {

private:
  static const double coeffs[];

  struct elbow_t {
    int16_t omega{};
    int16_t theta{};
  } params;

  PipeHandler pipe;

  ros::Subscriber subscriber;
  ros::ServiceServer test_service;
  ros::Publisher publisher;

public:
  ElbowHandler(ros::NodeHandle *node);
  void callback_elbow(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
};

#endif // ELBOW_HANDLER_H

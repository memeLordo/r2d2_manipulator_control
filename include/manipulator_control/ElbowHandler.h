#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
// #include <vector>

class ElbowHandler {

private:
  struct elbow_t {
  } params;
  // static const std::vector<double> coeffs;
  static const double coeffs[];

  PipeHandler pipeHandler;

  ros::Publisher publisher;
  ros::Subscriber subscriber;
  ros::ServiceServer test_service;

public:
  ElbowHandler(ros::NodeHandle *node);
  void callback_elbow(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
};

#endif // ELBOW_HANDLER_H

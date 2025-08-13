#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>
// #include <vector>

class ElbowHandler {

private:
  struct elbow_t {
  } params;
  ros::Publisher elbow_publisher;
  ros::Subscriber elbow_subscriber;
  static const double coeffs[];
  ros::ServiceServer test_service;
  // static const std::vector<double> coeffs;

public:
  ElbowHandler(ros::NodeHandle *node);
  void callback_elbow(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
};

#endif // ELBOW_HANDLER_H

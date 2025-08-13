#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "HornerPolynome.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

struct elbow_t {};

class ElbowHandler {

private:
  ros::Publisher elbow_publisher;
  ros::Subscriber elbow_subscriber;
  ros::ServiceServer test_service;
  const std::vector<double> coeffs{0.00024, 0.142, 20.9};

public:
  ElbowHandler(ros::NodeHandle *node);
  void callback_elbow(const std_msgs::Int64 &msg);
  bool callback_service(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);
};

#endif // ELBOW_HANDLER_H

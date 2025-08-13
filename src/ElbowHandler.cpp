#include "ElbowHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

const double ElbowHandler::coeffs[]{0.00024, 0.142, 20.9};

ElbowHandler::ElbowHandler(ros::NodeHandle *node) {
  elbow_subscriber = node->subscribe("/manipulator/elbow_input", 1000,
                                     &ElbowHandler::callback_elbow, this);
  elbow_publisher =
      node->advertise<std_msgs::Int64>("/manipulator/elbow_output", 10);

  test_service = node->advertiseService("/test_service",
                                        &ElbowHandler::callback_service, this);
}
void ElbowHandler::callback_elbow(const std_msgs::Int64 &msg) {
  // test_number += msg.data;
  // std_msgs::Int64 new_msg;
  // new_msg.data = test_number;
  // pub.publish(new_msg);
}
bool ElbowHandler::callback_service(std_srvs::SetBool::Request &req,
                                    std_srvs::SetBool::Response &res) {
  // if (req.data) {
  //   test_number = 0;
  //   res.success = true;
  //   res.message = "Counter has been successfully reset";
  // } else {
  //   res.success = false;
  //   res.message = "Counter has not been reset";
  // }

  return true;
}
// double get_angle(double radius) { return h_polynome(coeffs, radius); }

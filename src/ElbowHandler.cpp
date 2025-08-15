#include "ElbowHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

#define ELBOW_INPUT_NODE "/manipulator/elbow_input"
#define ELBOW_OUTPUT_NODE "/manipulator/elbow_output"

const double ElbowHandler::coeffs[]{0.00024, 0.142, 20.9};

ElbowHandler::ElbowHandler(ros::NodeHandle *node) : pipeHandler(node) {
  subscriber = node->subscribe(ELBOW_INPUT_NODE, 1000,
                               &ElbowHandler::callback_elbow, this);
  publisher = node->advertise<std_msgs::Int64>(ELBOW_OUTPUT_NODE, 10);

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

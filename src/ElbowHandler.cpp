#include "ElbowHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <ros/ros.h>

#define ELBOW_INPUT_NODE "/manipulator/elbow_input"
#define ELBOW_OUTPUT_NODE "/manipulator/elbow_output"

const double ElbowHandler::coeffs[]{0.00024, 0.142, 20.9};

ElbowHandler::ElbowHandler(ros::NodeHandle *node) : pipe(node) {
  subscriber = node->subscribe(ELBOW_INPUT_NODE, 1000,
                               &ElbowHandler::callback_elbow, this);
  publisher = node->advertise<r2d2_msg_pkg::DriverState>(ELBOW_OUTPUT_NODE, 10);
}
void ElbowHandler::callback_elbow(
    const r2d2_msg_pkg::DriverCommand::ConstPtr &msg) {
  // test_number += msg.data;
  // std_msgs::Int64 new_msg;
  // new_msg.data = test_number;
  // pub.publish(new_msg);
}

// double get_angle(double radius) { return h_polynome(coeffs, radius); }

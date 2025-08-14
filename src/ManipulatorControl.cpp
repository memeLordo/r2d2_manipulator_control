#include "ManipulatorControlHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>

ManipulatorControlHandler::ManipulatorControlHandler(ros::NodeHandle *node)
    : elbowHandler(node), shoulderHandler(node) {
  subscriber =
      node->subscribe("/manupulator/payload_input", 1000,
                      &ManipulatorControlHandler::callback_manipulator, this);
  publisher =
      node->advertise<std_msgs::Int64>("/manipulator/payload_output", 10);
  setup();
}
void ManipulatorControlHandler::setup() {
}
void ManipulatorControlHandler::callback_manipulator(
    const std_msgs::Int64 &msg) {}

#include "PayloadHandler.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <ros/ros.h>

#define PAYLOAD_OUTPUT_NODE "/manupulator/payload_output"

PayloadHandler::PayloadHandler(ros::NodeHandle *node) {
  subscriber = node->subscribe(PAYLOAD_OUTPUT_NODE, 1000,
                               &PayloadHandler::callback_payload, this);
}
void PayloadHandler::callback_payload(
    const r2d2_msg_pkg::DriverStateConstPtr &msg) {
  callback_force = msg->force;
}

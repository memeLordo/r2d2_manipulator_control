#include "PayloadHandler.h"
#include <ros/ros.h>

#define PAYLOAD_OUTPUT_NODE "/manupulator/payload_output"

PayloadHandler::PayloadHandler(ros::NodeHandle *node) {
  subscriber = node->subscribe(PAYLOAD_OUTPUT_NODE, 1000,
                               &PayloadHandler::callback_payload, this);
}

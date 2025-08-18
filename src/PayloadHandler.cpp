#include "PayloadHandler.h"
#include <ros/ros.h>

#define PAYLOAD_OUTPUT_NODE "/payload_output"

template <typename T> PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node) {
  subscriber = node->subscribe(PAYLOAD_OUTPUT_NODE, 1000,
                               &PayloadHandler::callback_payload, this);
}
template class PayloadHandler<>;

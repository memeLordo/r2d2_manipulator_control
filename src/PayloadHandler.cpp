#include "PayloadHandler.h"
#include <ros/node_handle.h>

template <typename T>
const std::string PayloadHandler<T>::OUTPUT_NODE = "/payload_output";

template <typename T> PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(OUTPUT_NODE, QUEUE_SIZE,
                                 &PayloadHandler::callbackPayload, this);
}
template class PayloadHandler<>;

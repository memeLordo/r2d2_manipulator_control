#include "PayloadHandler.hpp"
#include <ros/node_handle.h>

template <typename T>
PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node)
    : m_outputNode{"/payload_output"} {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &PayloadHandler::callbackPayload, this);
}
template class PayloadHandler<>;

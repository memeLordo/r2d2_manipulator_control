#include "PayloadHandler.hpp"
#include "utils/Config.hpp"
#include <ros/node_handle.h>

template <typename T>
PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node)
    : m_outputNode{config::payload::OUTPUT_NODE} {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &PayloadHandler::callbackPayload, this);
}
template class PayloadHandler<>;

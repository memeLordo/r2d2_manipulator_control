#include "PayloadHandler.h"
#include <ros/node_handle.h>

template <typename T>
PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node,
                                  const std::string &outputNode) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(outputNode, QUEUE_SIZE,
                                 &PayloadHandler::callbackPayload, this);
}
template class PayloadHandler<>;

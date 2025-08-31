#include "PipeHandler.h"
#include <ros/node_handle.h>

template <typename T>
PipeHandler<T>::PipeHandler(ros::NodeHandle *node,
                            const std::string &outputNode) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber =
      node->subscribe(outputNode, QUEUE_SIZE, &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

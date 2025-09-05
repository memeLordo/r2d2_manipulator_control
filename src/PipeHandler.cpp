#include "PipeHandler.hpp"

template <typename T>
PipeHandler<T>::PipeHandler(ros::NodeHandle *node)
    : m_outputNode{"/parameters/pipe"} {
  waitForTopic();
  m_subscriber =
      node->subscribe(m_outputNode, 10, &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

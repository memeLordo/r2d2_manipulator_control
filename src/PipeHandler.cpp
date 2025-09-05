#include "PipeHandler.hpp"

template <typename T>
PipeHandler<T>::PipeHandler(ros::NodeHandle *node)
    : m_outputNode{"/parameters/pipe"} {
  constexpr int QUEUE_SIZE = 8;
  waitForTopic();
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

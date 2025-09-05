#include "PipeHandler.hpp"

template <typename T>
PipeHandler<T>::PipeHandler(ros::NodeHandle *node)
    : m_outputTopic{"/parameters/pipe"} {
  waitForTopic();
  m_subscriber =
      node->subscribe(m_outputTopic, 10, &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

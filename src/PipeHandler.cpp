#include "PipeHandler.hpp"
#include "utils/Config.hpp"
#include <ros/node_handle.h>

template <typename T>
PipeHandler<T>::PipeHandler(ros::NodeHandle *node)
    : m_outputNode{config::pipe::OUTPUT_NODE} {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

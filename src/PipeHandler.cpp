#include "PipeHandler.h"
#include <ros/node_handle.h>

template <typename T>
const std::string PipeHandler<T>::OUTPUT_NODE = "/pipe_output";

template <typename T> PipeHandler<T>::PipeHandler(ros::NodeHandle *node) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(OUTPUT_NODE, QUEUE_SIZE,
                                 &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

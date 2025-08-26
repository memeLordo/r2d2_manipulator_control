#include "PipeHandler.h"
#include <ros/node_handle.h>

constexpr const char *PIPE_OUTPUT_NODE = "/parameters/pipe";

template <typename T> PipeHandler<T>::PipeHandler(ros::NodeHandle *node) {
  constexpr int QUEUE_SIZE = 8;
  subscriber = node->subscribe(PIPE_OUTPUT_NODE, QUEUE_SIZE,
                               &PipeHandler::callback_pipe, this);
}
template class PipeHandler<>;

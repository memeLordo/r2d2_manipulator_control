#include "PipeHandler.h"
#include <ros/node_handle.h>

#define PIPE_OUTPUT_NODE "/parameters/pipe"

template <typename T> PipeHandler<T>::PipeHandler(ros::NodeHandle *node) {
  subscriber = node->subscribe(PIPE_OUTPUT_NODE, 1000,
                               &PipeHandler::callback_pipe, this);
}
template class PipeHandler<>;

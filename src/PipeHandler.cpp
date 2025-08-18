#include "PipeHandler.h"
#include <ros/ros.h>

#define PIPE_OUTPUT_NOD "/parameters/pipe"

template <typename T> PipeHandler<T>::PipeHandler(ros::NodeHandle *node) {
  subscriber =
      node->subscribe(PIPE_OUTPUT_NOD, 1000, &PipeHandler::callback_pipe, this);
}
template class PipeHandler<>;

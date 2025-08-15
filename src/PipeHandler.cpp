#include "PipeHandler.h"
#include "r2d2_msg_pkg/PipeParameters.h"
#include <ros/ros.h>

#define PIPE_INPUT_NODE "/parameters/pipe"

PipeHandler::PipeHandler(ros::NodeHandle *node) {
  subscriber =
      node->subscribe(PIPE_OUTPUT_NOD, 1000, &PipeHandler::callback_pipe, this);
}
void PipeHandler::callback_pipe(
    const r2d2_msg_pkg::PipeParametersConstPtr &msg) {
  callback_params = pipe_t{msg->pipe_diam, msg->pipe_thickness};
}

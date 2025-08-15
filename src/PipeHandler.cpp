#include "PipeHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>

#define PIPE_INPUT_NODE "/pipe_subscriber_node"

PipeHandler::PipeHandler(ros::NodeHandle *node) {
  subscriber =
      node->subscribe(PIPE_INPUT_NODE, 1000, &PipeHandler::callback_pipe, this);
}
void PipeHandler::callback_pipe(const std_msgs::Int64 &msg) {
  // params = pipe_t{msg->pipe_diam, msg->pipe_thickness};
  callback_params =
      pipe_t{static_cast<uint16_t>(msg.data), static_cast<uint8_t>(msg.data)};
}

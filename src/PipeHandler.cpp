#include "PipeHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>

PipeHandler::PipeHandler(ros::NodeHandle *node) {
  pipe_subscriber = node->subscribe("/pipe_subscriber_node", 1000,
                                    &PipeHandler::callback_pipe, this);
}
void PipeHandler::callback_pipe(const std_msgs::Int64 &msg) {
  // params = pipe_t{msg->pipe_diam, msg->pipe_thickness};
  params =
      pipe_t{static_cast<uint16_t>(msg.data), static_cast<uint8_t>(msg.data)};
}
// double PipeHandler::get_radius() { return params.radius(); }

#ifndef PIPE_HANDLER_H
#define PIPE_HANDLER_H

#include <ros/ros.h>
#include <std_msgs/Int64.h>

struct pipe_t {
  uint16_t diameter;
  uint8_t thickness;

  double radius() const { return (double)diameter / 2.0 - thickness; }
};

class PipeHandler {
private:
  pipe_t params{};
  ros::Subscriber pipe_subscriber;

public:
  PipeHandler(ros::NodeHandle *node);
  void callback_pipe(const std_msgs::Int64 &msg);
  const pipe_t &get_params() const { return params; }
};

#endif // PIPE_HANDLER_H

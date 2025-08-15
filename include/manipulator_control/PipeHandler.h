#ifndef PIPE_HANDLER_H
#define PIPE_HANDLER_H

#include "r2d2_msg_pkg/PipeParameters.h"
#include <cstdint>
#include <ros/ros.h>

class PipeHandler {
private:
  struct pipe_t {
    uint16_t diameter{};
    uint8_t thickness{};
    double radius() const { return (double)diameter / 2.0 - thickness; }
  } callback_params{};

  ros::Subscriber subscriber;

public:
  PipeHandler(ros::NodeHandle *node);
  void callback_pipe(const r2d2_msg_pkg::PipeParametersConstPtr &msg);
  // const pipe_t &get_params() const { return params; }
  const double get_radius() const { return callback_params.radius(); };
};

#endif // PIPE_HANDLER_H

#ifndef PIPE_HANDLER_H
#define PIPE_HANDLER_H

#include "r2d2_msg_pkg/PipeParameters.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class PipeHandler {
private:
  struct pipe_t {
    uint16_t diameter{};
    uint8_t thickness{};
    T radius() const { return T{diameter} / T{2} - thickness; };
  } callback_params{};

  ros::Subscriber subscriber;

public:
  PipeHandler(ros::NodeHandle *node);
  void callback_pipe(const r2d2_msg_pkg::PipeParametersConstPtr &msg) {
    callback_params = pipe_t{msg->pipe_diam, msg->pipe_thickness};
  };
  T get_radius() const { return callback_params.radius(); };
};

#endif // PIPE_HANDLER_H

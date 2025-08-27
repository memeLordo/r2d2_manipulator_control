#ifndef PIPE_HANDLER_H
#define PIPE_HANDLER_H

#include "r2d2_msg_pkg/PipeParameters.h"
#include "utils/Debug.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class PipeHandler {
private:
  static const std::string OUTPUT_NODE;

  struct pipe_t {
    uint16_t diameter{};
    uint8_t thickness{};
    T radius() const { return (T)diameter / T{2} - thickness; };
  } m_callbackParams{};

  ros::Subscriber m_subscriber;

public:
  PipeHandler(ros::NodeHandle *node);

private:
  void callbackPipe(const r2d2_msg_pkg::PipeParametersConstPtr &msg) {
    m_callbackParams = pipe_t{msg->pipe_diam, msg->pipe_thickness};
  };

public:
  T getRadius() const {
    auto radius = m_callbackParams.radius();
    ROS_DEBUG_STREAM("Pipe::getRadius() : " << WHITE(radius));
    return radius;
  };
};

#endif // PIPE_HANDLER_H

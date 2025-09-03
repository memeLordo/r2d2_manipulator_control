#ifndef R2D2_PIPE_HANDLER_HPP
#define R2D2_PIPE_HANDLER_HPP

#include "r2d2_msg_pkg/PipeParameters.h"
#include "utils/Debug.hpp"
#include "utils/Types.hpp"
#include <ros/node_handle.h>

template <typename T = double> class PipeHandler {
private:
  static constexpr const char *s_name = "Pipe";

  const std::string m_outputNode;
  r2d2_types::upipe_t<T> m_callbackParams{};
  ros::Subscriber m_subscriber;

public:
  PipeHandler(ros::NodeHandle *node);

private:
  void callbackPipe(const r2d2_msg_pkg::PipeParametersConstPtr &msg) {
    m_callbackParams =
        r2d2_types::upipe_t<T>{msg->pipe_diam, msg->pipe_thickness};
  };

public:
  T getRadius() const {
    auto radius = m_callbackParams.radius();
    ROS_DEBUG_STREAM(s_name << "::getRadius() : " << WHITE(radius));
    return radius;
  };
};

#endif // R2D2_PIPE_HANDLER_HPP

#ifndef R2D2_PIPE_HANDLER_HPP
#define R2D2_PIPE_HANDLER_HPP

#include "r2d2_msg_pkg/PipeParameters.h"
#include "utils/Debug.hpp"
#include "utils/Types.hpp"
#include <ros/topic.h>

template <typename T = double> class PipeHandler {
private:
  static constexpr const char *s_name = "Pipe";

  const std::string m_outputNode;
  r2d2_type::upipe_t<T> m_callbackParams{};
  ros::Subscriber m_subscriber;

public:
  PipeHandler(ros::NodeHandle *node);

private:
  void callbackPipe(const r2d2_msg_pkg::PipeParametersConstPtr &msg) {
    m_callbackParams =
        r2d2_type::upipe_t<T>{msg->pipe_diam, msg->pipe_thickness};
  };

public:
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << s_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::PipeParameters>(m_outputNode);
  }
  T getRadius() const {
    auto radius = m_callbackParams.radius();
    ROS_DEBUG_STREAM(s_name << "::getRadius() : " << WHITE(radius));
    return radius;
  };
};

#endif // R2D2_PIPE_HANDLER_HPP

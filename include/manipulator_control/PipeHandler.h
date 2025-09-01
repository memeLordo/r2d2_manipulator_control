#ifndef PIPE_HANDLER_H
#define PIPE_HANDLER_H

#include "r2d2_msg_pkg/PipeParameters.h"
#include "utils/Debug.h"
#include "utils/Types.h"
#include <ros/node_handle.h>

template <typename T = double> class PipeHandler {
private:
  const std::string outputNode;

  r2d2_types::upipe_t<T> m_callbackParams{};

  ros::Subscriber m_subscriber;

public:
  PipeHandler(ros::NodeHandle *node, const std::string &outputNode);

private:
  void callbackPipe(const r2d2_msg_pkg::PipeParametersConstPtr &msg) {
    m_callbackParams =
        r2d2_types::upipe_t<T>{msg->pipe_diam, msg->pipe_thickness};
  };

public:
  T getRadius() const {
    auto radius = m_callbackParams.radius();
    ROS_DEBUG_STREAM("Pipe::getRadius() : " << WHITE(radius));
    return radius;
  };
};

#endif // PIPE_HANDLER_H

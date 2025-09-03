#ifndef PAYLOAD_HANDLER_H
#define PAYLOAD_HANDLER_H

#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class PayloadHandler {
private:
  static constexpr const char *s_name = "Payload";

  const std::string m_outputNode;
  int16_t m_callbackForce{};
  ros::Subscriber m_subscriber;

public:
  PayloadHandler(ros::NodeHandle *node);

private:
  void callbackPayload(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackForce = msg->force;
  };

public:
  T getForce() const {
    auto force = static_cast<T>(m_callbackForce);
    ROS_DEBUG_STREAM(s_name << "::getForce() : " << WHITE(force));
    return force;
  };
};

#endif // PIPE_HANDLER_H

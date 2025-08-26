#ifndef PAYLOAD_HANDLER_H
#define PAYLOAD_HANDLER_H

#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class PayloadHandler {
private:
  int16_t m_callbackForce;
  // TODO: add omega -  раскрутка щёток
  // считать желаемую скорость из manipulator_t

  ros::Subscriber m_subscriber;

public:
  PayloadHandler(ros::NodeHandle *node);

private:
  void callbackPayload(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackForce = msg->force;
  };

public:
  T getForce() const { return static_cast<T>(m_callbackForce); };
};

#endif // PIPE_HANDLER_H

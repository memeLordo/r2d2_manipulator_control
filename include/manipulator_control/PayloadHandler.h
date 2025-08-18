#ifndef PAYLOAD_HANDLER_H
#define PAYLOAD_HANDLER_H

#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class PayloadHandler {
private:
  int16_t callback_force;

  ros::Subscriber subscriber;

public:
  PayloadHandler(ros::NodeHandle *node);
  void callback_payload(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    callback_force = msg->force;
  };
  T get_force() const { return static_cast<T>(callback_force); };
};

#endif // PIPE_HANDLER_H

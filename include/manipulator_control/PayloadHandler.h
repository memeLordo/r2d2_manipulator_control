#ifndef PAYLOAD_HANDLER_H
#define PAYLOAD_HANDLER_H

#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/ros.h>

class PayloadHandler {
private:
  int16_t callback_force;

  ros::Subscriber subscriber;

public:
  PayloadHandler(ros::NodeHandle *node);
  void callback_payload(const r2d2_msg_pkg::DriverStateConstPtr &msg);
};

#endif // PIPE_HANDLER_H

#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/ros.h>

class ShoulderHandler {
private:
  static const double coeffs[];

  struct shoulder_t {
    int16_t omega{};
    int16_t theta{};
  } params;
  shoulder_t callback_params;

  PipeHandler pipe;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ShoulderHandler(ros::NodeHandle *node);
  void callback_shoulder(const r2d2_msg_pkg::DriverStateConstPtr &msg);
  // void get_input_speed(const r2d2_msg_pkg::DriverCommandConstPtr &msg);
  // void get_input_angle(const r2d2_msg_pkg::DriverCommandConstPtr &msg);
  void update_speed(const double *omega = nullptr);
  void update_angle(const double *theta = nullptr);
  double get_speed();
  double get_angle();
  double calc_angle(const double *theta = nullptr);
};

#endif // SHOULDER_HANDLER_H

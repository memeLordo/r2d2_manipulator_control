#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/ros.h>

class ShoulderHandler {
private:
  static const double coeffs[];
  static const double length;

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
  void callback_shoulder(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    callback_params = shoulder_t{msg->omega, msg->theta};
  };
  void update_speed() { params.omega = callback_params.omega; };
  void update_angle() { params.theta = callback_params.theta; };
  template <typename T> void update_speed(T omega) {
    params.omega = static_cast<int16_t>(omega);
  };
  template <typename T> void update_angle(T theta) {
    params.theta = static_cast<int16_t>(theta);
  };
  void update() {
    update_speed();
    update_angle();
  };
  template <typename T = double> T get_speed() const {
    return static_cast<T>(params.omega);
  };
  template <typename T = double> T get_angle() const {
    return static_cast<T>(params.theta);
  };
  template <typename T = double> T get_length() const {
    return static_cast<T>(length);
  };
  template <typename T = double> T calc_angle() { return 0; };
  template <typename T = double> T calc_angle(T theta) { return 0; };
  auto prepare_msg() const {
    r2d2_msg_pkg::DriverCommand msg;
    msg.omega = params.omega;
    msg.theta = params.theta;
    return msg;
  };
  void publish() { publisher.publish(prepare_msg()); };
};

#endif // SHOULDER_HANDLER_H

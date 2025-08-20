#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ShoulderHandler {

private:
  static const T coeffs[];
  static const T length;

  struct shoulder_t {
    int16_t omega{};
    int16_t theta{};
  } params;
  shoulder_t callback_params;

  const PipeHandler<T> &pipe;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

  bool needs_publish{false};

public:
  ShoulderHandler(ros::NodeHandle *node, const PipeHandler<T> &);
  void callback_shoulder(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    callback_params = shoulder_t{msg->omega, msg->theta};
  };
  void update_speed() { params.omega = callback_params.omega; };
  void update_angle() { params.theta = callback_params.theta; };
  void update_speed(T omega) { params.omega = static_cast<int16_t>(omega); };
  void update_angle(T theta) { params.theta = static_cast<int16_t>(theta); };
  void update() {
    update_speed();
    update_angle();
  };
  void set_publish_pending(bool pending = true) { needs_publish = pending; }
  void clear_publish_pending() { needs_publish = false; }
  r2d2_msg_pkg::DriverCommand prepare_msg() const {
    r2d2_msg_pkg::DriverCommand msg;
    msg.omega = params.omega;
    msg.theta = params.theta;
    return msg;
  };
  void publish() { publisher.publish(prepare_msg()); };
  T get_speed() const { return static_cast<T>(params.omega); };
  T get_angle() const { return static_cast<T>(params.theta); };
  T get_length() const { return static_cast<T>(length); };
  bool is_publish_pending() const { return needs_publish; }
  T calc_angle();
  T calc_angle(T theta);
};

#endif // SHOULDER_HANDLER_H

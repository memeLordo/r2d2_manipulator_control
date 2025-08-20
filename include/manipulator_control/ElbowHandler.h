#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ElbowHandler {

private:
  static const T coeffs[];
  static const T length;

  struct elbow_t {
    int16_t omega{};
    int16_t theta{};
  } params;
  elbow_t callback_params;

  const PipeHandler<T> &pipe;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ElbowHandler(ros::NodeHandle *node, const PipeHandler<T> &);
  void callback_elbow(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    callback_params = elbow_t{msg->omega, msg->theta};
  };
  void update_speed() { params.omega = callback_params.omega; };
  void update_angle() { params.theta = callback_params.theta; };
  void update_speed(T omega) { params.omega = static_cast<int16_t>(omega); };
  void update_angle(T theta) { params.theta = static_cast<int16_t>(theta); };
  void update() {
    update_speed();
    update_angle();
  };
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
  T calc_angle();
  T calc_angle(T theta);
};

#endif // ELBOW_HANDLER_H

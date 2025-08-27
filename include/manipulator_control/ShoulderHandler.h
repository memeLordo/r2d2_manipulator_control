#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Math.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ShoulderHandler {

private:
  static const T m_coeffs[];
  static const T m_length;

  template <typename Type> struct shoulder_t {
    Type omega{};
    Type theta{};
  };
  shoulder_t<T> m_params{};
  shoulder_t<int16_t> m_callbackParams{};

  const PipeHandler<T> &m_pipe;

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

  bool m_needsPublish{false};

public:
  ShoulderHandler(ros::NodeHandle *node, const PipeHandler<T> &);

private:
  void callbackShoulder(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams = shoulder_t<int16_t>{msg->omega, msg->theta};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    auto omega_ = r2d2_process::unwrap<int16_t>(m_params.omega);
    auto theta_ = r2d2_process::unwrap<int16_t>(m_params.theta);
    r2d2_msg_pkg::DriverCommand msg;
    msg.omega = omega_;
    msg.theta = theta_;
    return msg;
  };

public:
  void updateSpeed() {
    ROS_INFO("Update callback shoulder | omega : %f", m_callbackParams.omega);
    auto omega_ = r2d2_process::wrap<T>(m_callbackParams.omega);
    m_params.omega = omega_;
  };
  void updateAngle() {
    ROS_INFO("Update callback shoulder | theta : %f", m_callbackParams.theta);
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    m_params.theta = theta_;
  };
  void updateSpeed(T omega) {
    ROS_INFO("Update callback shoulder | omega : %f", omega);
    m_params.omega = omega;
  };
  void updateAngle(T theta) {
    m_params.theta = theta;
  };

  void setPublishPending(bool pending = true) {
    ROS_INFO("Set publish pending to %d", pending);
    m_needsPublish = pending;
  }
  void clearPublishPending() {
    ROS_INFO("Clear publish pending");
    m_needsPublish = false;
  }

  void publish() {
    if (m_params.omega != 0 || m_params.theta != 0) {
      m_publisher.publish(prepareMsg());
    }
  };
  bool isPublishPending() const {
    ROS_INFO("Get publish pending");
    return m_needsPublish;
  }

  T getSpeed() const {
    ROS_INFO("Get shoulder | speed : %f", m_params.omega);
    return m_params.omega;
  };
  T getAngle() const {
    ROS_INFO("Get shoulder | angle : %f", m_params.theta);
    return m_params.theta;
  };
  T getLength() const {
    ROS_INFO("Get shoulder | length : %f", m_length);
    return m_length;
  };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // SHOULDEN_HANDLER_H

#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ShoulderHandler {

private:
  static const T m_coeffs[];
  static const T m_length;

  struct shoulder_t {
    int16_t omega{};
    int16_t theta{};
  } m_params;
  shoulder_t m_callbackParams;

  const PipeHandler<T> &m_pipe;

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

  bool m_needsPublish{false};

public:
  ShoulderHandler(ros::NodeHandle *node, const PipeHandler<T> &);

private:
  void callbackShoulder(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams = shoulder_t{msg->omega, msg->theta};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    r2d2_msg_pkg::DriverCommand msg;
    msg.omega = m_params.omega;
    msg.theta = m_params.theta;
    return msg;
  };

public:
  void updateSpeed() {
    ROS_INFO("Update callback shoulder | omega : %f", m_callbackParams.omega);
    m_params.omega = m_callbackParams.omega;
  };
  void updateAngle() {
    ROS_INFO("Update callback shoulder | theta : %f", m_callbackParams.theta);
    m_params.theta = m_callbackParams.theta;
  };
  void updateSpeed(T omega) {
    ROS_INFO("Update callback shoulder | omega : %f", omega);
    m_params.omega = static_cast<int16_t>(omega);
  };
  void updateAngle(T theta) { m_params.theta = static_cast<int16_t>(theta); };

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
    return static_cast<T>(m_params.omega);
  };
  T getAngle() const {
    ROS_INFO("Get shoulder | angle : %f", m_params.theta);
    return static_cast<T>(m_params.theta);
  };
  T getLength() const {
    ROS_INFO("Get shoulder | length : %f", m_length);
    return m_length;
  };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // SHOULDEN_HANDLER_H

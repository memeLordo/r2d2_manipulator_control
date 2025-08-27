#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.h"
#include "utils/Math.h"
#include <cstdint>
#include <ros/console.h>
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
    ROS_DEBUG_STREAM("Prepare shoulder msg |"
                     << YELLOW(" omeha : ") << WHITE(omega_) << " "
                     << YELLOW(" theta : ") << WHITE(theta_));
    r2d2_msg_pkg::DriverCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.omega = omega_;
    msg.theta = theta_;
    msg.control_word = 10;
    return msg;
  };

public:
  void updateSpeed() {
    auto omega_ = r2d2_process::wrap<T>(m_callbackParams.omega);
    ROS_DEBUG_STREAM("Shoulder::updateSpeed() : " << WHITE(omega_));
    m_params.omega = omega_;
  };
  void updateAngle() {
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    ROS_DEBUG_STREAM("Shoulder::updateAngle() : " << WHITE(theta_));
    m_params.theta = theta_;
  };
  void updateSpeed(T omega) {
    ROS_DEBUG_STREAM("Shoulder::updateSpeed(" << WHITE(omega) << GREEN(")"));
    m_params.omega = omega;
  };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM("Shoulder::updateAngle(" << WHITE(theta) << GREEN(")"));
    m_params.theta = theta;
  };

  void setPublishPending(bool pending = true) {
    ROS_DEBUG_STREAM("Set publish pending to " << pending);
    m_needsPublish = pending;
  }
  void clearPublishPending() {
    ROS_DEBUG("Clear publish pending");
    m_needsPublish = false;
  }

  void publish() {
    ROS_DEBUG_STREAM_COND(m_params.omega == 0 && m_params.theta == 0,
                          RED("Shoulder : no update"));
    if (m_params.omega != 0 || m_params.theta != 0) {
      ROS_DEBUG_STREAM(BLUE("Shoulder::publish()"));
      m_publisher.publish(prepareMsg());
    }
  };
  bool isPublishPending() const {
    ROS_DEBUG("Get publish pending");
    return m_needsPublish;
  }

  T getSpeed() const {
    ROS_DEBUG_STREAM("Shoulder::getSpeed() : " << WHITE(m_params.omega));
    return m_params.omega;
  };
  T getAngle() const {
    ROS_DEBUG_STREAM("Shoulder::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  T getLength() const {
    ROS_DEBUG_STREAM("Shoulder::getLength() : " << WHITE(m_length));
    return m_length;
  };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // SHOULDEN_HANDLER_H

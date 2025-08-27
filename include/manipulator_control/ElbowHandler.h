#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.h"
#include "utils/Math.h"
#include <cstdint>
#include <ros/console.h>
#include <ros/node_handle.h>

template <typename T = double> class ElbowHandler {

private:
  static const T m_coeffs[];
  static const T m_length;

  template <typename Type> struct elbow_t {
    Type omega{};
    Type theta{};
  };
  elbow_t<T> m_params{};
  elbow_t<int16_t> m_callbackParams{};

  const PipeHandler<T> &m_pipe;

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

public:
  ElbowHandler(ros::NodeHandle *node, const PipeHandler<T> &);

private:
  void callbackElbow(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams = elbow_t<int16_t>{msg->omega, msg->theta};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    auto omega_ = r2d2_process::unwrap<int16_t>(m_params.omega);
    auto theta_ = r2d2_process::unwrap<int16_t>(m_params.theta);
    ROS_DEBUG_STREAM("Prepare elbow msg |"
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
    ROS_DEBUG_STREAM("Elbow::updateSpeed() : " << WHITE(omega_));
    m_params.omega = omega_;
  };
  void updateAngle() {
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    ROS_DEBUG_STREAM("Elbow::updateAngle() : " << WHITE(theta_));
    m_params.theta = theta_;
  };
  void updateSpeed(T omega) {
    ROS_DEBUG_STREAM("Elbow::updateSpeed(" << WHITE(omega) << ")");
    m_params.omega = omega;
  };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM("Elbow::updateAngle(" << WHITE(theta) << ")");
    m_params.theta = theta;
  };

  void publish() {
    ROS_DEBUG_STREAM_COND(m_params.omega == 0 && m_params.theta == 0,
                          RED("Elbow : no update"));
    if (m_params.omega != 0 || m_params.theta != 0) {
      ROS_DEBUG_STREAM(BLUE("Elbow::publish()"));
      m_publisher.publish(prepareMsg());
    }
  };

  T getSpeed() const {
    ROS_DEBUG_STREAM("Elbow::getSpeed() : " << WHITE(m_params.omega));
    return m_params.omega;
  };
  T getAngle() const {
    ROS_DEBUG_STREAM("Elbow::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  T getLength() const {
    ROS_DEBUG_STREAM("Elbow::getLength() : " << WHITE(m_length));
    return m_length;
  };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // ELBOW_HANDLER_H

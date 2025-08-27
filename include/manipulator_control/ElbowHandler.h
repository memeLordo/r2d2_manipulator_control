#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "ros/console.h"
#include "utils/Math.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ElbowHandler {

private:
  static const T m_coeffs[];
  static const T m_length;

  struct elbow_t {
    int16_t omega{};
    int16_t theta{};
  } m_params;
  elbow_t m_callbackParams;

  const PipeHandler<T> &m_pipe;

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

public:
  ElbowHandler(ros::NodeHandle *node, const PipeHandler<T> &);

private:
  void callbackElbow(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams = elbow_t{msg->omega, msg->theta};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    ROS_INFO("Prepare elbow msg | omeha : %f  theta : %f", m_params.omega,
             m_params.theta);
    auto omega_ = r2d2_process::unwrap<int16_t>(m_params.omega);
    auto theta_ = r2d2_process::unwrap<int16_t>(m_params.theta);
    r2d2_msg_pkg::DriverCommand msg;
    msg.omega = omega_;
    msg.theta = theta_;
    return msg;
  };

public:
  void updateSpeed() {
    ROS_INFO("Update callback elbow | omega : %f", m_callbackParams.omega);
    auto omega_ = r2d2_process::wrap<T>(m_callbackParams.omega);
    m_params.omega = omega_;
  };
  void updateAngle() {
    ROS_INFO("Update callback elbow | theta : %f", m_callbackParams.theta);
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    m_params.theta = theta_;
  };
  void updateSpeed(T omega) {
    ROS_INFO("Update callback elbow | omega : %f", omega);
    m_params.omega = static_cast<int16_t>(omega);
  };
  void updateAngle(T theta) {
    ROS_INFO("Update callback elbow | theta : %f", theta);
    m_params.theta = static_cast<int16_t>(theta);
  };

  void publish() {
    if (m_params.omega != 0 || m_params.theta != 0) {
      m_publisher.publish(prepareMsg());
    }
  };

  T getSpeed() const {
    ROS_INFO("Get elbow | omega : %f", m_params.omega);
    return static_cast<T>(m_params.omega);
  };
  T getAngle() const {
    ROS_INFO("Get elbow | theta : %f", m_params.theta);
    return static_cast<T>(m_params.theta);
  };
  T getLength() const {
    ROS_INFO("Get elbow | length : %f", m_length);
    return static_cast<T>(m_length);
  };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // ELBOW_HANDLER_H

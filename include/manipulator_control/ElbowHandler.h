#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
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
    r2d2_msg_pkg::DriverCommand msg;
    msg.omega = m_params.omega;
    msg.theta = m_params.theta;
    return msg;
  };

public:
  void updateSpeed() { m_params.omega = m_callbackParams.omega; };
  void updateAngle() { m_params.theta = m_callbackParams.theta; };
  void updateSpeed(T omega) { m_params.omega = static_cast<int16_t>(omega); };
  void updateAngle(T theta) { m_params.theta = static_cast<int16_t>(theta); };

  void publish() { m_publisher.publish(prepareMsg()); };

  T getSpeed() const { return static_cast<T>(m_params.omega); };
  T getAngle() const { return static_cast<T>(m_params.theta); };
  T getLength() const { return static_cast<T>(m_length); };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // ELBOW_HANDLER_H

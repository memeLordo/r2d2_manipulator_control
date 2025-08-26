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
  void updateSpeed() { m_params.omega = m_callbackParams.omega; };
  void updateAngle() { m_params.theta = m_callbackParams.theta; };
  void updateSpeed(T omega) { m_params.omega = static_cast<int16_t>(omega); };
  void updateAngle(T theta) { m_params.theta = static_cast<int16_t>(theta); };

  void setPublishPending(bool pending = true) { m_needsPublish = pending; }
  void clearPublishPending() { m_needsPublish = false; }

  void publish() { m_publisher.publish(prepareMsg()); };
  bool isPublishPending() const { return m_needsPublish; }

  T getSpeed() const { return static_cast<T>(m_params.omega); };
  T getAngle() const { return static_cast<T>(m_params.theta); };
  T getLength() const { return m_length; };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // SHOULDEN_HANDLER_H

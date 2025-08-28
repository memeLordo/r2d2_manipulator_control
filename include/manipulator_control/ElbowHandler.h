#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.h"
#include "utils/Math.h"
#include "utils/Types.h"
#include <ros/console.h>
#include <ros/node_handle.h>

template <typename T = double> class ElbowHandler {

private:
  static const std::string INPUT_NODE;
  static const std::string OUTPUT_NODE;

  static const T m_coeffs[];
  static const T m_length;
  static const T m_speed;

  r2d2_types::elbow_t<T, r2d2_commands::ControlType> m_params{};
  r2d2_types::elbow16_t m_callbackParams{};

  const PipeHandler<T> &m_pipe;

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

public:
  ElbowHandler(ros::NodeHandle *node, const PipeHandler<T> &);

private:
  void callbackElbow(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams =
        r2d2_types::elbow16_t{msg->omega, msg->theta, msg->control_word};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    auto omega_ = m_speed;
    auto theta_ = r2d2_process::unwrap<int16_t>(m_params.theta);
    auto control_word_ = static_cast<uint16_t>(m_params.control_word);
    ROS_DEBUG_STREAM("Prepare elbow msg |"
                     << YELLOW(" omeha : ") << WHITE(omega_) << " "
                     << YELLOW(" theta : ") << WHITE(theta_)
                     << YELLOW(" control_word : ") << WHITE(control_word_));
    r2d2_msg_pkg::DriverCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.omega = omega_;
    msg.theta = theta_;
    msg.control_word = control_word_;
    return msg;
  };

public:
  // void updateSpeed() {
  //   // auto omega_ = m_callbackParams.omega;
  //   ROS_DEBUG_STREAM("Elbow::updateSpeed("
  //                    << YELLOW("callback = " << m_callbackParams.omega)
  //                    << ") : " << WHITE(m_speed));
  //   m_params.omega = m_speed;
  // };
  void updateAngle() {
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    ROS_DEBUG_STREAM("Elbow::updateAngle("
                     << YELLOW("callback = " << m_callbackParams.theta)
                     << ") : " << WHITE(theta_));
    m_params.theta = theta_;
  };
  };
  // void updateSpeed(T omega) {
  //   ROS_DEBUG_STREAM("Elbow::updateSpeed(omega = " << WHITE(omega) << ")");
  //   m_params.omega = omega;
  // };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM("Elbow::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
  };

  void publish() {
    ROS_DEBUG_STREAM(BLUE("Elbow::publish()"));
    m_publisher.publish(prepareMsg());
  };

  std::string getInputNode() const { return INPUT_NODE; };
  std::string getOutputNode() const { return OUTPUT_NODE; };

  };
  // T getSpeed() const {
  //   ROS_DEBUG_STREAM("Elbow::getSpeed() : " << WHITE(m_params.omega));
  //   return m_params.omega;
  // };
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

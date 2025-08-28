#ifndef SHOULDER_HANDLER_H
#define SHOULDER_HANDLER_H

#include "PipeHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.h"
#include "utils/Math.h"
#include "utils/Types.h"
#include <ros/console.h>
#include <ros/node_handle.h>

template <typename T = double> class ShoulderHandler {

private:
  static const std::string OUTPUT_NODE;
  static const std::string INPUT_NODE;

  static const T m_coeffs[];
  static const T m_length;
  static const T m_speed;

  r2d2_types::shoulder_t<T, r2d2_commands::ControlType> m_params{};
  r2d2_types::shoulder16_t m_callbackParams{};

  const PipeHandler<T> &m_pipe;

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

public:
  ShoulderHandler(ros::NodeHandle *node, const PipeHandler<T> &);

private:
  void callbackShoulder(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams =
        r2d2_types::shoulder16_t{msg->omega, msg->theta, msg->control_word};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    auto omega_ = m_speed;
    auto theta_ = r2d2_process::unwrap<int16_t>(m_params.theta);
    auto control_word_ = static_cast<uint16_t>(m_params.control_word);
    ROS_DEBUG_STREAM("Prepare shoulder msg |"
                     << YELLOW(" omeha : ") << WHITE(omega_) << " "
                     << YELLOW(" theta : ") << WHITE(theta_) << " "
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
  //   ROS_DEBUG_STREAM("Shoulder::updateSpeed("
  //                    << YELLOW("callback = " << m_callbackParams.omega)
  //                    << ") : " << WHITE(m_speed));
  //   m_params.omega = m_speed;
  // };
  void updateAngle() {
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    ROS_DEBUG_STREAM("Shoulder::updateAngle("
                     << YELLOW("callback = " << m_callbackParams.theta)
                     << ") : " << WHITE(theta_));
    m_params.theta = theta_;
  };
  void updateControlWord() {
    auto control_word_ =
        static_cast<r2d2_commands::ControlType>(m_callbackParams.control_word);
    ROS_DEBUG_STREAM("Shoulder::updateControlWord("
                     << YELLOW("callback = " << m_callbackParams.control_word)
                     << ") ");
    m_params.control_word = control_word_;
  };
  // void updateSpeed(T omega) {
  //   ROS_DEBUG_STREAM("Shoulder::updateSpeed(omega = " << WHITE(omega) <<
  //   ")"); m_params.omega = omega;
  // };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM("Shoulder::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
  };
  void holdControl() {
    m_params.control_word = r2d2_commands::ControlType::HOLD;
    ROS_DEBUG_STREAM(BLUE("Shoulder::set control_word to " << YELLOW(
                              static_cast<uint16_t>(m_params.control_word))));
  };
  void setControlByAngle() {
    m_params.control_word = r2d2_commands::ControlType::CONTROL_ANGLE;
    ROS_DEBUG_STREAM(BLUE("Shoulder::set control_word to " << YELLOW(
                              static_cast<uint16_t>(m_params.control_word))));
  };
  void setControlBySpeed() {
    m_params.control_word = r2d2_commands::ControlType::CONTROL_SPEED;
    ROS_DEBUG_STREAM(BLUE("Shoulder::set control_word to " << YELLOW(
                              static_cast<uint16_t>(m_params.control_word))));
  };

  void publish() {
    ROS_DEBUG_STREAM(BLUE("Shoulder::publish()"));
    m_publisher.publish(prepareMsg());
  };

  std::string getInputNode() const { return INPUT_NODE; };
  std::string getOutputNode() const { return OUTPUT_NODE; };

  bool isOnHold() const {
    return m_params.control_word == r2d2_commands::ControlType::HOLD;
  };
  bool isControlByAngle() const {
    return m_params.control_word == r2d2_commands::ControlType::CONTROL_ANGLE;
  };
  bool isControlBySpeed() const {
    return m_params.control_word == r2d2_commands::ControlType::CONTROL_SPEED;
  };

  T getSpeed() const {
    auto speed_ = static_cast<T>(m_params.omega);
    ROS_DEBUG_STREAM("Shoulder::getSpeed() : " << WHITE(speed_));
    return speed_;
  };
  T getAngle() const {
    auto angle_ = static_cast<T>(m_params.theta);
    ROS_DEBUG_STREAM("Shoulder::getAngle() : " << WHITE(angle_));
    return angle_;
  };
  // r2d2_commands::ControlType getControlWord() const {
  //   ROS_DEBUG_STREAM("Shoulder::getControlWord() : "
  //                    << WHITE(static_cast<uint16_t>(m_params.control_word)));
  //   return m_params.control_word;
  // };
  T getLength() const {
    ROS_DEBUG_STREAM("Shoulder::getLength() : " << WHITE(m_length));
    return m_length;
  };

  T calcAngle();
  T calcAngle(T theta);
};

#endif // SHOULDEN_HANDLER_H

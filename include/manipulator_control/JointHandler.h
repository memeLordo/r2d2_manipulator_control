#ifndef ELBOW_HANDLER_H
#define ELBOW_HANDLER_H

#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.h"
#include "utils/Math.h"
#include "utils/Types.h"
#include <ros/console.h>
#include <ros/node_handle.h>

template <typename T = double> class JointHandler {

private:
  r2d2_types::elbow_t<T, r2d2_commands::ControlType> m_params{};
  r2d2_types::elbow16_t m_callbackParams{};

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

  const std::string m_inputNode;
  const std::string m_outputNode;
  const std::vector<T> m_coeffs;
  const T m_length;
  const T m_speed;

public:
  JointHandler() = default;
  JointHandler(ros::NodeHandle *node, const std::string &inputNode,
               const std::string &outputNode, const T &length, const T &speed,
               const std::vector<T> &coeffs);

private:
  void callbackJoint(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
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
  //
  // private:
  //   std::array<T, N> setCoeffsFrom(const std::array<T, N> &arr) {
  //     return std::reverse_copy(arr.begin(), arr.end(), m_coeffs.begin());
  //   }

public:
  // void updateSpeed() {
  //   // auto omega_ = m_callbackParams.omega;
  //   ROS_DEBUG_STREAM("Joint::updateSpeed("
  //                    << YELLOW("callback = " << m_callbackParams.omega)
  //                    << ") : " << WHITE(m_speed));
  //   m_params.omega = m_speed;
  // };
  void updateAngle() {
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    ROS_DEBUG_STREAM("Joint::updateAngle("
                     << YELLOW("callback = " << m_callbackParams.theta)
                     << ") : " << WHITE(theta_));
    m_params.theta = theta_;
  };
  void updateControlWord() {
    auto control_word_ =
        static_cast<r2d2_commands::ControlType>(m_callbackParams.control_word);
    ROS_DEBUG_STREAM("Joint::updateControlWord("
                     << YELLOW("callback = " << m_callbackParams.control_word)
                     << ") ");
    m_params.control_word = control_word_;
  };
  // void updateSpeed(T omega) {
  //   ROS_DEBUG_STREAM("Joint::updateSpeed(omega = " << WHITE(omega) << ")");
  //   m_params.omega = omega;
  // };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM("Joint::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
  };
  void updateAngleByRadius(T radius) { updateAngle(calcAngle(radius)); };
  void setHoldControl() {
    m_params.control_word = r2d2_commands::ControlType::HOLD;
    ROS_DEBUG_STREAM(BLUE("Joint::set control_word to " << YELLOW(
                              static_cast<uint16_t>(m_params.control_word))));
  };
  void setControlByAngle() {
    m_params.control_word = r2d2_commands::ControlType::CONTROL_ANGLE;
    ROS_DEBUG_STREAM(BLUE("Joint::set control_word to " << YELLOW(
                              static_cast<uint16_t>(m_params.control_word))));
  };
  void setControlBySpeed() {
    m_params.control_word = r2d2_commands::ControlType::CONTROL_SPEED;
    ROS_DEBUG_STREAM(BLUE("Joint::set control_word to " << YELLOW(
                              static_cast<uint16_t>(m_params.control_word))));
  };

  void publish() {
    ROS_DEBUG_STREAM(BLUE("Joint::publish()"));
    m_publisher.publish(prepareMsg());
  };

  std::string getInputNode() const { return m_inputNode; };
  std::string getOutputNode() const { return m_outputNode; };

  bool checkAngleDiff(T margin = 0.1) const {
    auto angle_ = getAngle();
    auto input_angle_ = getInputAngle();
    bool res = r2d2_math::abs(angle_ - input_angle_) < margin;
    ROS_DEBUG_STREAM("Joint::checkAngleDiff() : " << WHITE(res));
    return res;
  };
  // T getSpeed() const {
  //   ROS_DEBUG_STREAM("Joint::getSpeed() : " << WHITE(m_params.omega));
  //   return m_params.omega;
  // };
  T getAngle() const {
    ROS_DEBUG_STREAM("Joint::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  T getInputAngle() const {
    ROS_DEBUG_STREAM(
        "Joint::getInputAngle() : " << WHITE(m_callbackParams.theta));
    return r2d2_process::wrap<T>(m_callbackParams.theta);
  };
  // r2d2_commands::ControlType getControlWord() const {
  //   ROS_DEBUG_STREAM("Joint::getControlWord() : "
  //                    << WHITE(static_cast<uint16_t>(m_params.control_word)));
  //   return m_params.control_word;
  // };
  T getLength() const {
    ROS_DEBUG_STREAM("Joint::getLength() : " << WHITE(m_length));
    return m_length;
  };
  T getRadius() const {
    T radius_ = getLength() * r2d2_math::sin(getAngle());
    ROS_DEBUG_STREAM("Joint::getRadius() : " << WHITE(radius_));
    return radius_;
  };

  T calcAngle(T radius);
};

#endif // ELBOW_HANDLER_H

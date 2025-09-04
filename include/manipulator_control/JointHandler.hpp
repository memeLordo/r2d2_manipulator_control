#ifndef R2D2_JOINT_HANDLER_HPP
#define R2D2_JOINT_HANDLER_HPP

#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Config.hpp"
#include "utils/Debug.hpp"
#include "utils/Math.hpp"
#include "utils/Types.hpp"
#include <ros/console.h>
#include <ros/node_handle.h>

template <typename T = double> class JointHandler {

private:
  r2d2_types::elbow_t<T, r2d2_commands::ControlType> m_params{};
  r2d2_types::elbow16_t m_callbackParams{};

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

  const std::string m_name;
  const std::string m_inputNode;
  const std::string m_outputNode;
  const std::vector<T> m_coeffs;
  const T m_length;
  const T m_speed;

public:
  JointHandler() = default;
  JointHandler(ros::NodeHandle *node, const std::string &name,
               const std::string &inputNode, const std::string &outputNode,
               const T &length, const T &speed, const std::vector<T> &coeffs);

private:
  void callbackJoint(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams =
        r2d2_types::elbow16_t{msg->omega, msg->theta, msg->control_word};
  };

  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    auto omega_ = m_speed;
    auto theta_ = r2d2_process::unwrap<int16_t>(m_params.theta);
    auto control_word_ = static_cast<uint16_t>(m_params.control_word);
    ROS_DEBUG_STREAM(m_name << "::prepareMsg() |" << YELLOW(" omeha : ")
                            << WHITE(omega_) << " " << YELLOW(" theta : ")
                            << WHITE(theta_) << YELLOW(" control_word : ")
                            << WHITE(control_word_));
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
  //   ROS_DEBUG_STREAM("Joint::updateSpeed("
  //                    << YELLOW("callback = " << m_callbackParams.omega)
  //                    << ") : " << WHITE(m_speed));
  //   m_params.omega = m_speed;
  // };
  void updateAngle() {
    auto theta_ = r2d2_process::wrap<T>(m_callbackParams.theta);
    ROS_DEBUG_STREAM(m_name << "::updateAngle("
                            << YELLOW("callback = " << m_callbackParams.theta)
                            << ") : " << WHITE(theta_));
    m_params.theta = theta_;
  };
  void updateControlWord() {
    auto control_word_ =
        static_cast<r2d2_commands::ControlType>(m_callbackParams.control_word);
    ROS_DEBUG_STREAM(m_name
                     << "::updateControlWord("
                     << YELLOW("callback = " << m_callbackParams.control_word)
                     << ") ");
    m_params.control_word = control_word_;
  };
  // void updateSpeed(T omega) {
  //   ROS_DEBUG_STREAM("Joint::updateSpeed(omega = " << WHITE(omega) << ")");
  //   m_params.omega = omega;
  // };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM(m_name << "::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
  };
  void updateAngleByRadius(T radius) { updateAngle(calcAngle(radius)); };
  void setHoldControl() {
    m_params.control_word = r2d2_commands::ControlType::HOLD;
    ROS_DEBUG_STREAM(
        BLUE(m_name << "::set control_word to "
                    << YELLOW(static_cast<uint16_t>(m_params.control_word))));
  };
  void setControlByAngle() {
    m_params.control_word = r2d2_commands::ControlType::CONTROL_ANGLE;
    ROS_DEBUG_STREAM(
        BLUE(m_name << "::set control_word to "
                    << YELLOW(static_cast<uint16_t>(m_params.control_word))));
  };
  void setControlBySpeed() {
    m_params.control_word = r2d2_commands::ControlType::CONTROL_SPEED;
    ROS_DEBUG_STREAM(
        BLUE(m_name << "::set control_word to "
                    << YELLOW(static_cast<uint16_t>(m_params.control_word))));
  };

  void publish() {
    ROS_DEBUG_STREAM(BLUE(m_name << "::publish()"));
    m_publisher.publish(prepareMsg());
  };

  std::string getInputNode() const { return m_inputNode; };
  std::string getOutputNode() const { return m_outputNode; };

  // T getSpeed() const {
  //   ROS_DEBUG_STREAM("Joint::getSpeed() : " << WHITE(m_params.omega));
  //   return m_params.omega;
  // };
  T getAngle() const {
    ROS_DEBUG_STREAM(m_name << "::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  T getInputAngle() const {
    ROS_DEBUG_STREAM(m_name << "::getInputAngle() : "
                            << WHITE(m_callbackParams.theta));
    return r2d2_process::wrap<T>(m_callbackParams.theta);
  };
  // r2d2_commands::ControlType getControlWord() const {
  //   ROS_DEBUG_STREAM("Joint::getControlWord() : "
  //                    << WHITE(static_cast<uint16_t>(m_params.control_word)));
  //   return m_params.control_word;
  // };
  T getLength() const {
    ROS_DEBUG_STREAM(m_name << "::getLength() : " << WHITE(m_length));
    return m_length;
  };
  T getRadius() const {
    T radius_ = getLength() * r2d2_math::sin(getAngle());
    ROS_DEBUG_STREAM(m_name << "::getRadius() : " << WHITE(radius_));
    return radius_;
  };

  T calcAngle(T radius);
};

template <typename T> class ShoulderHandler : public JointHandler<T> {

public:
  ShoulderHandler(ros::NodeHandle *node)
      : JointHandler<T>(node, "Shoulder", "/shoulder_input", "/shoulder_output",
                        config::shoulder::length, // длина плеча
                        config::shoulder::speed,  // скорость плеча
                        config::shoulder::coeffs) // коэффициенты плеча
  {}
};

template <typename T = double> class ElbowHandler : public JointHandler<T> {

public:
  ElbowHandler(ros::NodeHandle *node)
      : JointHandler<T>(node, "Elbow", "/elbow_input", "/elbow_output",
                        config::elbow::length, // длина локтя
                        config::elbow::speed,  // скорость локтя
                        config::elbow::coeffs) // коэффициенты локтя
  {}
};

#endif // R2D2_JOINT_HANDLER_HPP

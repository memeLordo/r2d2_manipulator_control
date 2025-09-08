#ifndef R2D2_JOINT_HANDLER_HPP
#define R2D2_JOINT_HANDLER_HPP

#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.hpp"
#include "utils/IConfigJson.hpp"
#include "utils/Math.hpp"
#include "utils/Types.hpp"
#include <ros/topic.h>

template <typename T> class JointConfig : private IConfigJson<T> {
protected:
  const std::string m_name;
  const std::string m_inputTopic;
  const std::string m_outputTopic;
  const T m_length;
  const T m_speed;
  const T m_angleOffset;
  const T m_angleTolerance;
  const std::vector<T> m_coeffs;

  explicit JointConfig(const std::string &name)
      : IConfigJson<T>(name), m_name{name}, m_inputTopic{getInputTopic()},
        m_outputTopic{getOutputTopic()}, m_length(this->getParam("length")),
        m_speed(this->getParam("speed")),
        m_angleOffset(this->getParam("angle_offset")),
        m_angleTolerance(this->getParam("angle_tolerance")),
        m_coeffs(this->getVector("coeffs")) {};

private:
  std::string getInputTopic() const {
    return "/" + this->lower(m_name) + "_input";
  };
  std::string getOutputTopic() const {
    return "/" + this->lower(m_name) + "_output";
  };
};

template <typename T = double> class JointHandler : private JointConfig<T> {
private:
  using JointConfig<T>::m_name;
  using JointConfig<T>::m_inputTopic;
  using JointConfig<T>::m_outputTopic;
  using JointConfig<T>::m_length;
  using JointConfig<T>::m_speed;
  using JointConfig<T>::m_angleOffset;
  using JointConfig<T>::m_angleTolerance;
  using JointConfig<T>::m_coeffs;

  r2d2_type::joint_t<T, r2d2_commands::ControlType> m_params{};
  r2d2_type::joint16_t m_callbackParams{};

  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;

  bool m_needsTolerance{false};
  bool m_needsRefresh{true};

public:
  JointHandler() = default;
  JointHandler(ros::NodeHandle *node, const std::string &name);

private:
  void callbackJoint(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams =
        r2d2_type::joint16_t{msg->omega, msg->theta, msg->control_word};
  };

  T getAngleTolerance() const {
    return m_needsTolerance ? m_angleTolerance : 0;
  };
  bool checkAngleDiff(const T radius) {
    const T angleDiff_{std::abs(getAngle() - getTargetAngle(radius))};
    const bool needsAngleControl_{angleDiff_ < getAngleTolerance()};
    ROS_DEBUG_STREAM(
        CYAN(m_name << "::needsAngleControl_ = " << needsAngleControl_));
    return needsAngleControl_;
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
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << m_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputTopic);
  };
  void updateAngle() {
    T theta_{r2d2_process::wrap<T>(m_callbackParams.theta)};
    ROS_DEBUG_STREAM(m_name << "::updateAngle("
                            << YELLOW("callback = " << m_callbackParams.theta)
                            << ") : " << WHITE(theta_));
    m_params.theta = theta_;
    setHoldControl();
  };
  void updateAngle(T theta) {
    ROS_DEBUG_STREAM(m_name << "::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
    setControlByAngle();
  };
  void updateAngleByDiff(short diff, const T dTheta = 0.1) {
    const T theta_{diff * dTheta};
    ROS_DEBUG_STREAM(m_name << "::changeAngleBy(diff = " << WHITE(diff)
                            << ", dTheta = " << WHITE(dTheta) << ")");
    m_params.theta += theta_;
    setControlByAngle();
  };
  void updateAngleByRadius(T radius) {
    if (checkAngleDiff(radius) && m_needsRefresh)
      updateAngle(getTargetAngle(radius));
    else
      updateAngle();
  };
  void enableTolerance() {
    ROS_DEBUG_STREAM(m_name << "::enableTolerance()");
    m_needsTolerance |= true;
  };
  void setRefresh() {
    ROS_DEBUG_STREAM(m_name << "::setRefresh()");
    m_needsRefresh |= true;
  };
  void stopRefresh() {
    ROS_DEBUG_STREAM(m_name << "::stopRefresh()");
    m_needsRefresh &= false;
  };
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

  T getRadius() const { return m_length * r2d2_math::sin(getAngle()); };

  T getAngle() const {
    ROS_DEBUG_STREAM(m_name << "::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };

  T getTargetAngle(T radius);
};

template <typename T = double> class ShoulderHandler : public JointHandler<T> {
public:
  ShoulderHandler(ros::NodeHandle *node);
};
template <typename T = double> class ElbowHandler : public JointHandler<T> {
public:
  ElbowHandler(ros::NodeHandle *node);
};

#endif // R2D2_JOINT_HANDLER_HPP

#ifndef R2D2_JOINT_HANDLER_HPP
#define R2D2_JOINT_HANDLER_HPP

#include <ros/topic.h>

#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/ConfigJson.hpp"
#include "utils/Debug.hpp"
#include "utils/Math.hpp"
#include "utils/Polynome.hpp"
#include "utils/Types.hpp"

template <typename T>
class JointConfig : public IConfigJsonMap<r2d2_type::config::joint_t, T> {
 protected:
  const std::string m_name;
  const std::string m_inputTopic{"/" + r2d2_json::lower(m_name) + "_input"};
  const std::string m_outputTopic{"/" + r2d2_json::lower(m_name) + "_output"};
  const r2d2_type::config::joint_t<T> m_config;

 protected:
  explicit JointConfig(const std::string &name,
                       const std::string &fileName = "joints")
      : IConfigJsonMap<r2d2_type::config::joint_t, T>{fileName},
        m_name{name},
        m_config{this->getParams(r2d2_json::lower(name))} {};
};

template <typename T = double>
class JointHandler : public JointConfig<T> {
 private:
  using ControlType = r2d2_commands::ControlType;
  using JointConfig<T>::m_name;
  using JointConfig<T>::m_inputTopic;
  using JointConfig<T>::m_outputTopic;
  using JointConfig<T>::m_config;
  r2d2_type::callback::joint_t<T> m_params{};
  r2d2_type::callback::joint16_t m_callbackParams{};
  ros::Subscriber m_subscriber;
  ros::Publisher m_publisher;
  bool m_needsAngleControl{true};

 public:
  JointHandler() = default;
  explicit JointHandler(ros::NodeHandle *node, const std::string &name)
      : JointConfig<T>(name) {
    waitForTopic();
    m_subscriber =
        node->subscribe(m_outputTopic, 1, &JointHandler::callbackJoint, this);
    m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputTopic, 1);
  };
  ~JointHandler() {
    ROS_DEBUG_STREAM(RED("~JointHandler()"));
    m_publisher.shutdown();
    m_subscriber.shutdown();
  };

 private:
  void callbackJoint(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams = r2d2_type::callback::joint16_t{msg->omega, msg->theta,
                                                      msg->control_word};
  };

 protected:
  bool needsAngleControl(const T theta) {
    m_needsAngleControl =
        std::abs(getAngle() - theta) > m_config.angle_tolerance;
    ROS_DEBUG_STREAM(
        CYAN(m_name << "::needsAngleControl_ = " << m_needsAngleControl));
    return m_needsAngleControl;
  };
  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    const auto omega_{m_config.speed};
    const auto theta_{r2d2_process::unwrap<int16_t>(m_params.theta)};
    const auto control_word_{static_cast<uint16_t>(m_params.control_word)};
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
  void setAngle(const T theta) {
    ROS_DEBUG_STREAM(m_name << "::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
  };
  void incrementAngleBy(short diff, const T dTheta = 0.1f) {
    const T theta_{diff * dTheta};
    ROS_DEBUG_STREAM(m_name << "::changeAngleBy(diff = " << WHITE(diff)
                            << ", dTheta = " << WHITE(dTheta) << ")");
    m_params.theta += theta_;
  };
  void setAngleByRadius(const T radius) {
    setAngle(getTargetAngle(radius));
    setControlWord(ControlType::CONTROL_ANGLE);
  };
  void updateAngleByRadius(const T radius, const bool needsUpdate = true) {
    setAngle(getCallbackAngle());
    ROS_DEBUG_STREAM(CYAN(m_name << "::needsUpdate = " << needsUpdate));
    if (!needsUpdate) return;

    const T targetAngle_{getTargetAngle(radius)};
    if (needsAngleControl(targetAngle_)) {
      setAngle(targetAngle_);
      setControlWord(ControlType::CONTROL_ANGLE);
      return;
    }
    setControlWord(ControlType::HOLD);
  };
  void setControlWord(ControlType control_word) {
    if (m_params.control_word == control_word) return;
    m_params.control_word = control_word;
    ROS_DEBUG_STREAM(BLUE(m_name
                          << "::set control_word to "
                          << YELLOW(static_cast<int>(m_params.control_word))));
  };
  void publish() {
    ROS_DEBUG_STREAM(BLUE(m_name << "::publish()"));
    m_publisher.publish(prepareMsg());
  };
  bool needsAngleControl() const { return m_needsAngleControl; };
  T getRadius() const {
    const T radius_{m_config.length * r2d2_math::sin(getAngle())};
    // ROS_DEBUG_STREAM(m_name << "::getRadius() : " << WHITE(radius_));
    return radius_;
  };
  T getAngle() const {
    ROS_DEBUG_STREAM(m_name << "::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  T getCallbackAngle() {
    const T theta_{r2d2_process::wrap<T>(m_callbackParams.theta)};
    ROS_DEBUG_STREAM(m_name << YELLOW("::getCallbackAngle() : ")
                            << WHITE(theta_));
    return theta_;
  };
  T getTargetAngle(T radius) const {
    const T theta_{horner::polynome(m_config.coeffs, radius) -
                   m_config.angle_offset};
    ROS_DEBUG_STREAM(m_name << "::calcAngle(radius = " << WHITE(radius)
                            << ") : " << WHITE(theta_));
    return r2d2_math::max<T>(theta_, 0);
  };
};

template <typename T = double>
class ShoulderHandler : public JointHandler<T> {
 public:
  ShoulderHandler(ros::NodeHandle *node) : JointHandler<T>(node, "Shoulder") {};
};
template <typename T = double>
class ElbowHandler : public JointHandler<T> {
 public:
  ElbowHandler(ros::NodeHandle *node) : JointHandler<T>(node, "Elbow") {};
};
#endif  // R2D2_JOINT_HANDLER_HPP

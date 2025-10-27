#ifndef INCLUDE_MANIPULATOR_CONTROL_JOINTHANDLER_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_JOINTHANDLER_HPP_

#include <ros/topic.h>

#include "TopicAwait.hpp"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "r2d2_utils_pkg/Collections.hpp"
#include "r2d2_utils_pkg/Debug.hpp"
#include "r2d2_utils_pkg/Json.hpp"
#include "r2d2_utils_pkg/Math.hpp"
#include "r2d2_utils_pkg/Polynome.hpp"
#include "r2d2_utils_pkg/Strings.hpp"
#include "r2d2_utils_pkg/Types.hpp"

template <typename T>
class JointConfig : private IJsonConfigMap<r2d2_type::config::joint_t, T> {
 protected:
  const std::string m_name;
  const std::string m_inputTopic;
  const std::string m_outputTopic;
  const r2d2_type::config::joint_t<T> m_config;

 protected:
  explicit JointConfig(std::string_view name,
                       std::string_view fileName = "joints")
      : IJsonConfigMap<r2d2_type::config::joint_t, T>{fileName},
        m_name{r2d2_string::upper(name, 0, 1)},
        m_inputTopic{"/" + std::string{name} + "_input"},
        m_outputTopic{"/" + std::string{name} + "_output"},
        m_config{this->getParams(name)} {};
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
  volatile bool m_needsControl{true};

 public:
  JointHandler() = default;
  explicit JointHandler(ros::NodeHandle* node, const std::string& name)
      : JointConfig<T>(name) {
    ROS_DEBUG_STREAM(MAGENTA(m_name + "Handler()"));
    waitForTopic<r2d2_msg_pkg::DriverState>(m_name, m_outputTopic);
    m_subscriber =
        node->subscribe(m_outputTopic, 1, &JointHandler::callbackJoint, this);
    m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputTopic, 1);
  };
  ~JointHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~" + m_name + "Handler()"));
    m_publisher.shutdown();
    m_subscriber.shutdown();
  };

 private:
  void callbackJoint(const r2d2_msg_pkg::DriverStateConstPtr& msg) {
    m_callbackParams = r2d2_type::callback::joint16_t{msg->omega, msg->theta,
                                                      msg->control_word};
  };

 protected:
  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    const auto omega_{m_config.speed};
    const auto theta_{r2d2_process::Angle::wrap<int16_t>(m_params.theta)};
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
  void publish() {
    ROS_DEBUG_STREAM(BLUE(m_name << "::publish()"));
    m_publisher.publish(prepareMsg());
  };
  void updateControlFlag(const T radius) {
    m_needsControl =
        r2d2_math::abs(getCallbackAngle() - getTargetAngle(radius)) >
        getAngleTolerance();
    ROS_DEBUG_STREAM(
        CYAN(m_name << "::needsAngleControl_ = " << m_needsControl));
  };
  void setControlFlag(const bool needsControl) {
    m_needsControl = needsControl;
  };
  void resetControlFlag() { setControlFlag(false); };
  void setControlWord(const ControlType control_word) {
    if (m_params.control_word == control_word) return;
    m_params.control_word = control_word;
    ROS_DEBUG_STREAM(BLUE(m_name
                          << "::set control_word to "
                          << YELLOW(static_cast<int>(m_params.control_word))));
  };
  void setAngle(const T theta) {
    // ROS_DEBUG_STREAM(m_name << "::updateAngle(theta = " << WHITE(theta) <<
    // ")");
    m_params.theta = theta;
  };
  void setCallbackAngle() {
    ROS_DEBUG_STREAM(m_name << "::setCallbackAngle()");
    setAngle(getCallbackAngle());
    setControlWord(ControlType::HOLD);
  };
  void resetAngle() {
    if (!m_needsControl) return;
    ROS_DEBUG_STREAM(m_name << "::resetAngle()");
    setAngle(0);
    setControlWord(ControlType::CONTROL_ANGLE);
  };
  void setAngleByRadius(const T radius) {
    if (!m_needsControl) return;
    ROS_DEBUG_STREAM(m_name << "::setAngleByRadius(" << WHITE(radius) << ")");
    setAngle(getTargetAngle(radius));
    setControlWord(ControlType::CONTROL_ANGLE);
  };
  void incrementAngleBy(const int diff, const T thetaStep = T{0.01}) {
    const T theta_{diff * thetaStep};
    ROS_DEBUG_STREAM(m_name << "::incrementAngleBy(diff = " << WHITE(diff)
                            << ", thetaStep = " << WHITE(thetaStep) << ")");
    m_params.theta += theta_;
  };

  [[nodiscard]] bool needsControl() const { return m_needsControl; };
  [[nodiscard]] T getAngle() const {
    ROS_DEBUG_STREAM(m_name << "::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  [[nodiscard]] T getAngleTolerance() const {
    return m_needsControl ? 0.1 : m_config.angle_tolerance;
  };
  [[nodiscard]] T getCallbackAngle() const {
    const T theta_{r2d2_process::Angle::unwrap<T>(m_callbackParams.theta)};
    ROS_DEBUG_STREAM(m_name << YELLOW("::getCallbackAngle() : ")
                            << WHITE(theta_));
    return theta_;
  };
  [[nodiscard]] T getRadius() const {
    const T radius_{m_config.length * r2d2_math::sin(getCallbackAngle())};
    ROS_DEBUG_STREAM(m_name << "::getRadius() : " << WHITE(radius_));
    return radius_;
  };
  [[nodiscard]] T getTargetAngle(T radius) const {
    const T theta_{horner::polynome(m_config.coeffs, radius) -
                   m_config.angle_offset};
    ROS_DEBUG_STREAM(m_name << "::calcAngle(radius = " << WHITE(radius)
                            << ") : " << WHITE(theta_));
    return r2d2_math::max<T>(theta_, 0);
  };
};

template <typename T = double>
class JointHandlerVector final
    : public NamedHandlerVector<std::vector, JointHandler, T> {
 public:
  template <typename... Args>
  JointHandlerVector(ros::NodeHandle* node, Args&&... names)
      : NamedHandlerVector<std::vector, JointHandler, T>(
            node, std::forward<Args>(names)...){};

 public:
  void publish() { this->call_each(&JointHandler<T>::publish); };
  void resetControl() { this->call_each(&JointHandler<T>::resetControl); };
  void resetAngle() { this->call_each(&JointHandler<T>::resetAngle); };
  void setCallbackAngle() {
    this->call_each(&JointHandler<T>::setCallbackAngle);
  };
  void setAngleByRadius(const T radius) {
    this->call_each(&JointHandler<T>::updateAngleByRadius, radius);
  };
  [[nodiscard]] bool needsControlAny() const {
    return std::any_of(this->cbegin(), this->cend(),
                       [](auto& obj) { return obj.needsControl(); });
  };
  [[nodiscard]] bool needsControlAll() const {
    return std::all_of(this->cbegin(), this->cend(),
                       [](auto& obj) { return obj.needsControl(); });
  };
  [[nodiscard]] T getRadius() const {
    auto radiuses_{this->get_each(&JointHandler<T>::getRadius)};
    return std::accumulate(radiuses_.cbegin(), radiuses_.cend(), T{0});
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_JOINTHANDLER_HPP_

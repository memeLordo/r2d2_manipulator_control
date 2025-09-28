#ifndef R2D2_JOINT_HANDLER_HPP
#define R2D2_JOINT_HANDLER_HPP

#include <ros/topic.h>

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
  volatile bool m_needsAngleControl{true};
  volatile bool m_needsUpdate{true};

 public:
  JointHandler() = default;
  explicit JointHandler(ros::NodeHandle* node, const std::string& name)
      : JointConfig<T>(name) {
    waitForTopic();
    m_subscriber =
        node->subscribe(m_outputTopic, 1, &JointHandler::callbackJoint, this);
    m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputTopic, 1);
  };
  ~JointHandler() {
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
  bool needsAngleControl(const T theta) {
    m_needsAngleControl =
        r2d2_math::abs(getAngle() - theta) > m_config.angle_tolerance;
    ROS_DEBUG_STREAM(
        CYAN(m_name << "::needsAngleControl_ = " << m_needsAngleControl));
    return m_needsAngleControl;
  };
  [[nodiscard]] r2d2_msg_pkg::DriverCommand prepareMsg() const {
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
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << m_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputTopic);
  };
  void setAngle(const T theta) {
    ROS_DEBUG_STREAM(m_name << "::updateAngle(theta = " << WHITE(theta) << ")");
    m_params.theta = theta;
  };
  void incrementAngleBy(const int8_t diff, const T dTheta = 0.1f) {
    const T theta_{diff * dTheta};
    ROS_DEBUG_STREAM(m_name << "::changeAngleBy(diff = " << WHITE(diff)
                            << ", dTheta = " << WHITE(dTheta) << ")");
    m_params.theta += theta_;
  };
  void setAngleByRadius(const T radius) {
    setAngle(getTargetAngle(radius));
    setControlWord(ControlType::CONTROL_ANGLE);
  };
  void updateAngleByRadius(const T radius) {
    setAngle(getCallbackAngle());
    ROS_DEBUG_STREAM(CYAN(m_name << "::needsUpdate = " << m_needsUpdate));
    if (!m_needsUpdate) return;

    if (const T targetAngle_{getTargetAngle(radius)};
        needsAngleControl(targetAngle_)) {
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

  [[nodiscard]] bool needsAngleControl() const { return m_needsAngleControl; };
  [[nodiscard]] T getRadius() const {
    return m_config.length * r2d2_math::sin(m_params.theta);
  };
  [[nodiscard]] T getAngle() const {
    ROS_DEBUG_STREAM(m_name << "::getAngle() : " << WHITE(m_params.theta));
    return m_params.theta;
  };
  [[nodiscard]] T getCallbackAngle() const {
    const T theta_{r2d2_process::Angle::unwrap<T>(m_callbackParams.theta)};
    ROS_DEBUG_STREAM(m_name << YELLOW("::getCallbackAngle() : ")
                            << WHITE(theta_));
    return theta_;
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
class ShoulderHandler : public JointHandler<T> {
 public:
  ShoulderHandler(ros::NodeHandle* node) : JointHandler<T>(node, "shoulder") {};
};
template <typename T = double>
class ElbowHandler : public JointHandler<T> {
 public:
  ElbowHandler(ros::NodeHandle* node) : JointHandler<T>(node, "elbow") {};
};

template <typename T = double>
class JointHandlerCollection : public NamedHandlerCollection<JointHandler, T> {
 public:
  template <typename... Args>
  JointHandlerCollection(ros::NodeHandle* node, Args&&... names)
      : NamedHandlerCollection<JointHandler, T>(node, names...){};

 public:
  void publish() { this->call_each(&JointHandler<T>::publish); };
  void updateAngleByRadius(const T radius) {
    this->call_each(&JointHandler<T>::updateAngleByRadius, radius);
  };
  [[nodiscard]] bool needAngleControlAny() const {
    return std::any_of(this->cbegin(), this->cend(),
                       [](auto& obj) { return obj.needsAngleControl(); });
  };
  [[nodiscard]] bool needAngleControlAll() const {
    return std::all_of(this->cbegin(), this->cend(),
                       [](auto& obj) { return obj.needsAngleControl(); });
  };
  [[nodiscard]] T getRadius() const {
    auto radiuses_{this->get_each(&JointHandler<T>::getRadius)};
    return std::accumulate(radiuses_.begin(), radiuses_.end(), T{0});
  };
};
#endif  // R2D2_JOINT_HANDLER_HPP

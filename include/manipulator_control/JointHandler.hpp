#ifndef INCLUDE_MANIPULATOR_CONTROL_JOINTHANDLER_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_JOINTHANDLER_HPP_

#include <ros/topic.h>

#include "TopicAwait.hpp"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "r2d2_utils_pkg/Collections.hpp"
#include "r2d2_utils_pkg/Json.hpp"
#include "r2d2_utils_pkg/Logging/Custom.hpp"
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
  /**
   * @brief   Constructs a JointConfig object with the specified joint name and
   *          configuration file.
   *
   * @param   name     The name of the joint (will be capitalized for display)
   * @param   fileName The name of the JSON configuration file
   *                   (default: "joints")
   *
   * @details Initializes the joint configuration by loading parameters from the
   *          JSON file, sets up input and output topic names based on the joint
   *          name.
   */
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

  /**
   * @brief   Constructs a JointHandler and initializes ROS subscribers and
   *          publishers.
   *
   * @param   node Pointer to the ROS node handle
   * @param   name The name of the joint to handle
   *
   * @details Waits for the driver state topic to become available, then
   *          subscribes to the output topic and advertises the input command
   *          topic.
   */
  explicit JointHandler(ros::NodeHandle* node, const std::string& name)
      : JointConfig<T>(name) {
    ROS_DEBUG_STREAM(MAGENTA(m_name + "Handler()"));
    waitForTopic<r2d2_msg_pkg::DriverState>(m_name, m_outputTopic);
    m_subscriber =
        node->subscribe(m_outputTopic, 1, &JointHandler::callbackJoint, this);
    m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputTopic, 1);
  };

  /**
   * @brief   Destructor that shuts down ROS publishers and subscribers.
   */
  ~JointHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~" + m_name + "Handler()"));
    m_publisher.shutdown();
    m_subscriber.shutdown();
  };

 private:
  /**
   * @brief   Callback function for receiving driver state messages.
   *
   * @param   msg The driver state message containing omega, theta, and
   *          control_word
   *
   * @details Updates the internal callback parameters with the latest joint
   *          state.
   */
  void callbackJoint(const r2d2_msg_pkg::DriverStateConstPtr& msg) {
    m_callbackParams = r2d2_type::callback::joint16_t{msg->omega, msg->theta,
                                                      msg->control_word};
  };

 protected:
  /**
   * @brief   Prepares a DriverCommand message with current joint parameters.
   *
   * @return  A DriverCommand message ready to be published
   *
   * @details Wraps the target angle, sets the speed from config, and includes
   *          the control word.
   */
  r2d2_msg_pkg::DriverCommand prepareMsg() const {
    const auto omega_{m_config.speed};
    const auto theta_{r2d2_process::Angle::wrap<int16_t>(m_params.theta)};
    const auto control_word_{static_cast<uint16_t>(m_params.control_word)};
    ROS_DEBUG_NAMED_COLORED_VOID_C(m_name, ANSI_YELLOW, omega_, theta_,
                                   control_word_);
    r2d2_msg_pkg::DriverCommand msg;
    msg.header.stamp = ros::Time::now();
    msg.omega = omega_;
    msg.theta = theta_;
    msg.control_word = control_word_;
    return msg;
  };

 public:
  /**
   * @brief   Publishes the prepared driver command message.
   */
  void publish() const { m_publisher.publish(prepareMsg()); };

  /**
   * @brief   Updates the control flag based on the difference between current
   *          and target angles.
   *
   * @param   radius The current radius value used to calculate the target angle
   *
   * @details Sets m_needsControl to true if the angle difference exceeds the
   *          tolerance threshold.
   */
  void updateControlFlag(const T radius) {
    m_needsControl =
        r2d2_math::abs(getCallbackAngle() - getTargetAngle(radius)) >
        getAngleTolerance();
    ROS_DEBUG_NAMED_COLORED_VARS_C(m_name, ANSI_CYAN, m_needsControl);
  };

  /**
   * @brief   Sets the control flag to the specified value.
   *
   * @param   needsControl Boolean indicating whether control is needed
   */
  void setControlFlag(const bool needsControl) {
    m_needsControl = needsControl;
  };

  /**
   * @brief   Resets the control flag to false.
   */
  void resetControlFlag() { setControlFlag(false); };

  /**
   * @brief   Sets the control word for the joint driver.
   *
   * @param   control_word The control type (e.g., HOLD, CONTROL_ANGLE)
   *
   * @details Only updates if the control word has changed to avoid unnecessary
   *          logging.
   */
  void setControlWord(const ControlType control_word) {
    if (m_params.control_word == control_word) return;
    m_params.control_word = control_word;
    ROS_DEBUG_NAMED_COLORED_VARS_C(m_name, ANSI_BLUE, control_word);
  };

  /**
   * @brief   Sets the target angle for the joint.
   *
   * @param   theta The target angle value
   */
  void setAngle(const T theta) { m_params.theta = theta; };

  /**
   * @brief   Sets the angle to the current callback angle and switches to HOLD
   *          mode.
   * @details Synchronizes the target angle with the current joint position.
   */
  void setCallbackAngle() {
    setAngle(getCallbackAngle());
    setControlWord(ControlType::HOLD);
  };

  /**
   * @brief   Resets the angle to zero and switches to CONTROL_ANGLE mode.
   *
   * @details Only performs the reset if control is needed (m_needsControl is
   *          true).
   */
  void resetAngle() {
    if (!m_needsControl) return;
    ROS_DEBUG_NAMED_VOID_C(m_name, "");
    setAngle(0);
    setControlWord(ControlType::CONTROL_ANGLE);
  };

  /**
   * @brief   Sets the angle based on the provided radius value.
   *
   * @param   radius The radius value used to calculate the target angle
   *
   * @details Only updates if control is needed. Calculates target angle using
   *          polynomial coefficients.
   */
  void setAngleByRadius(const T radius) {
    if (!m_needsControl) return;
    ROS_DEBUG_NAMED_VOID_C(m_name, radius);
    setAngle(getTargetAngle(radius));
    setControlWord(ControlType::CONTROL_ANGLE);
  };

  /**
   * @brief   Increments the current angle by a calculated delta.
   *
   * @param   diff The difference multiplier
   * @param   thetaStep The step size multiplier (default: 1)
   *
   * @details Calculates the increment as diff * thetaStep and adds it to the
   *          current angle.
   */
  void incrementAngleBy(const T diff, const T thetaStep = T{1}) {
    const T theta_{diff * thetaStep};
    ROS_DEBUG_NAMED_VOID_C(m_name, diff, thetaStep);
    m_params.theta += theta_;
  };

  /**
   * @brief   Checks if the joint needs control.
   *
   * @return  True if control is needed, false otherwise
   */
  [[nodiscard]] bool needsControl() const { return m_needsControl; };

  /**
   * @brief   Gets the current target angle.
   *
   * @return  The target angle value
   */
  [[nodiscard]] T getAngle() const {
    ROS_DEBUG_NAMED_VARS_C(m_name, m_params.theta);
    return m_params.theta;
  };

  /**
   * @brief   Gets the current angle from the callback data.
   *
   * @return  The unwrapped angle value from the latest driver state message
   */
  [[nodiscard]] T getCallbackAngle() const {
    const T theta_{r2d2_process::Angle::unwrap<T>(m_callbackParams.theta)};
    ROS_DEBUG_NAMED_FUNC_C(m_name, theta_, "");
    return theta_;
  };

  /**
   * @brief   Calculates the current radius based on the joint angle.
   *
   * @return  The radius value calculated as length * sin(angle)
   */
  [[nodiscard]] T getRadius() const {
    const T radius_{m_config.length * r2d2_math::sin(getCallbackAngle())};
    ROS_DEBUG_NAMED_FUNC_C(m_name, radius_, "");
    return radius_;
  };

  /**
   * @brief   Calculates the target angle for a given radius using polynomial
   *          coefficients.
   *
   * @param   radius The radius value to calculate the angle for
   * @return  The target angle, clamped to be non-negative
   *
   * @details Uses Horner's method to evaluate the polynomial and subtracts the
   *          angle offset.
   */
  [[nodiscard]] T getTargetAngle(T radius) const {
    const T theta_{horner::polynome(m_config.coeffs, radius) -
                   m_config.angle_offset};
    const T res_{r2d2_math::max<T>(theta_, 0)};
    ROS_DEBUG_NAMED_FUNC_C(m_name, res_, radius);
    return res_;
  };

  /**
   * @brief   Gets the angle tolerance threshold for control decisions.
   *
   * @param   minTolerance Minimum tolerance value (default: 0.1)
   * @return  The angle tolerance - minimum if control is needed, config value
   *          otherwise
   */
  [[nodiscard]] T getAngleTolerance(const T minTolerance = T{0.1}) const {
    return m_needsControl ? minTolerance : m_config.angle_tolerance;
  };
};

template <typename T = double>
class JointHandlerVector final
    : public NamedHandlerVector<std::vector, JointHandler, T> {
 public:
  /**
   * @brief   Constructs a vector of joint handlers.
   *
   * @param   node Pointer to the ROS node handle
   * @param   names Variadic list of joint names to create handlers for
   */
  template <typename... Args>
  JointHandlerVector(ros::NodeHandle* node, Args&&... names)
      : NamedHandlerVector<std::vector, JointHandler, T>(
            node, std::forward<Args>(names)...){};

 public:
  /**
   * @brief   Publishes commands for all joints in the vector.
   */
  void publish() { this->call_each(&JointHandler<T>::publish); };

  /**
   * @brief   Sets all joints to their callback angles (synchronizes with
   * current positions).
   */
  void setCallbackAngle() {
    this->call_each(&JointHandler<T>::setCallbackAngle);
  };

  /**
   * @brief   Resets the control flag for all joints.
   */
  void resetControlFlag() {
    this->call_each(&JointHandler<T>::resetControlFlag);
  };

  /**
   * @brief   Resets angles for all joints.
   */
  void resetAngle() { this->call_each(&JointHandler<T>::resetAngle); };

  /**
   * @brief   Checks if any joint needs control.
   *
   * @return  True if at least one joint needs control, false otherwise
   */
  [[nodiscard]] bool needsControlAny() const {
    return std::any_of(this->cbegin(), this->cend(),
                       [](auto& obj) { return obj.needsControl(); });
  };

  /**
   * @brief   Checks if all joints need control.
   *
   * @return  True if all joints need control, false otherwise
   */
  [[nodiscard]] bool needsControlAll() const {
    return std::all_of(this->cbegin(), this->cend(),
                       [](auto& obj) { return obj.needsControl(); });
  };

  /**
   * @brief   Calculates the total radius from all joints.
   *
   * @return  The sum of all joint radii
   */
  [[nodiscard]] T getRadius() const {
    auto radiuses_{this->get_each(&JointHandler<T>::getRadius)};
    return std::accumulate(radiuses_.cbegin(), radiuses_.cend(), T{0});
  };

  /**
   * @brief   Sets angles for all joints based on the provided radius.
   *
   * @param   radius The radius value to use for angle calculation
   */
  void setAngleByRadius(const T radius) {
    this->call_each(&JointHandler<T>::setAngleByRadius, radius);
  };

  /**
   * @brief   Updates control flags for all joints based on the provided radius.
   *
   * @param   radius The radius value used to calculate target angles
   */
  void updateControlFlag(const T radius) {
    this->call_each(&JointHandler<T>::updateControlFlag, radius);
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_JOINTHANDLER_HPP_

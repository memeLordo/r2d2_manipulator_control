#ifndef INCLUDE_MANIPULATOR_CONTROL_MANIPULATORCONTROL_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_MANIPULATORCONTROL_HPP_

#include "JointHandler.hpp"
#include "PayloadHandler.hpp"
#include "PipeHandler.hpp"
#include "r2d2_utils_pkg/Json.hpp"
#include "r2d2_utils_pkg/Types.hpp"

template <typename T>
class ManipulatorConfig
    : private IJsonConfigMap<r2d2_type::config::nozzle_t, T> {
 protected:
  r2d2_state::WorkModePair m_workMode{};
  r2d2_state::NozzleTypePair m_nozzleType{};
  r2d2_type::config::nozzle_t<T> m_config;

 protected:
  /**
   * @brief   Constructs a ManipulatorConfig object with the specified
   *          configuration file.
   *
   * @param   fileName The name of the JSON configuration file
   *                   (default: "nozzles")
   */
  explicit ManipulatorConfig(const std::string& fileName = "nozzles")
      : IJsonConfigMap<r2d2_type::config::nozzle_t, T>{fileName} {};

 protected:
  /**
   * @brief   Updates the configuration based on the current nozzle type.
   *
   * @details Reloads configuration parameters from the JSON file for the
   *          current nozzle type.
   */
  void updateConfig() { m_config = this->getParams(m_nozzleType.key); };

 public:
  /**
   * @brief   Sets the work mode for the manipulator.
   *
   * @param   value The work mode value to set
   * @return        True if the mode was set successfully,
   *                false if the mode is unknown
   *
   * @details Updates the internal work mode state and validates the mode.
   */
  template <typename U>
  bool setMode(const U& value) {
    ROS_DEBUG_STREAM("setMode(val=" << WHITE(static_cast<int>(value)) << ")");
    m_workMode.updateType(value);
    if (m_workMode.key.empty()) {
      ROS_ERROR_STREAM("Got unknown work mode!");
      return false;
    }
    return true;
  };

  /**
   * @brief   Sets the nozzle type and updates the configuration.
   *
   * @param   value The nozzle type value to set
   * @return        True if the nozzle type was set successfully,
   *                false if the type is unknown
   *
   * @details Updates the nozzle type and reloads configuration parameters.
   */
  template <typename U>
  bool setNozzle(const U& value) {
    ROS_DEBUG_STREAM("setNozzle(val=" << WHITE(static_cast<int>(value)) << ")");
    m_nozzleType.updateType(value);
    if (m_nozzleType.key.empty()) {
      ROS_ERROR_STREAM("Got unknown nozzle type!");
      return false;
    }
    updateConfig();
    return true;
  };

  /**
   * @brief   Resets the work mode to NONE.
   */
  void resetMode() {
    ROS_DEBUG("Reset mode");
    m_workMode.updateType(r2d2_state::WorkMode::NONE);
  };
};

template <typename T = double>
class ManipulatorControlHandler final : public ManipulatorConfig<T> {
 private:
  using ManipulatorConfig<T>::m_workMode;
  using ManipulatorConfig<T>::m_nozzleType;
  using ManipulatorConfig<T>::m_config;
  PipeHandler<T> m_pipe;
  PayloadHandler<T> m_payload;
  JointHandlerVector<T> m_joints;
  JointHandler<T>& m_shoulder{m_joints("shoulder")};
  JointHandler<T>& m_elbow{m_joints("elbow")};
  ros::Timer m_timer;
  volatile bool m_needsSetup{true};

 public:
  /**
   * @brief   Constructs a ManipulatorControlHandler and initializes all
   *          components.
   *
   * @param   node Pointer to the ROS node handle
   *
   * @details Initializes pipe, payload, and joint handlers, and sets up the
   *          control timer.
   */
  explicit ManipulatorControlHandler(ros::NodeHandle* node);

  /**
   * @brief   Destructor that stops the control timer.
   */
  ~ManipulatorControlHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~ManipulatorControlHandler()"));
    m_timer.stop();
  };

 private:
  /**
   * @brief   Timer callback function that processes manipulator control based
   *          on work mode.
   *
   * @param   event The timer event (unused)
   *
   * @details Handles different work modes: SETUP, AUTO, and STOP.
   */
  void callbackManipulator(const ros::TimerEvent&);

  /**
   * @brief   Processes the setup phase of manipulator control.
   *
   * @param   radius The current pipe radius
   * @param   force The current payload force
   *
   * @details Sets joint angles based on radius and checks if setup is complete.
   */
  void processSetup(const T radius, const T force);

  /**
   * @brief   Processes automatic control mode.
   *
   * @param   force The current payload force
   *
   * @details Adjusts joint angles based on force feedback and current radius.
   */
  void processControl(const T force);

  /**
   * @brief   Processes the stop phase, resetting joints to zero position.
   *
   * @details Resets all joints to zero angle when no control is needed.
   */
  void processStop();

 protected:
  /**
   * @brief   Checks if setup is still needed based on angle and force control
   *          status.
   *
   * @param   force The current payload force
   *
   * @details Updates m_needsSetup flag based on whether all joints need angle
   *          control and if force is below target threshold.
   */
  void checkSetup(const T force) {
    const bool needsAngleControl_{m_joints.needsControlAll()};
    const bool needsForceControl_{force < getTargetForce()};
    m_needsSetup = needsAngleControl_ && needsForceControl_;
    ROS_DEBUG_COLORED_VARS_C(ANSI_CYAN, needsAngleControl_, needsForceControl_,
                             m_needsSetup);
  };

  /**
   * @brief   Updates the payload control flag based on force tolerance.
   *
   * @param   force The current payload force
   *
   * @details Sets payload control flag if force exceeds tolerance threshold.
   */
  void updateControlFlag(const T force) {
    m_payload.setControl(r2d2_math::abs(force) > getForceTolerance());
  };

 public:
  /**
   * @brief   Gets the base radius from configuration.
   *
   * @return  The base radius value (r0) from nozzle configuration
   */
  [[nodiscard]] T getRadius() const {
    const T radius_{m_config.r0};
    ROS_DEBUG_STREAM("Base radius : " << WHITE(radius_));
    return radius_;
  };

  /**
   * @brief   Calculates the current total radius (base + joint contributions).
   *
   * @return  The sum of base radius and all joint radii
   */
  [[nodiscard]] T getCurrentRadius() const {
    const T currentRadius_{getRadius() + m_joints.getRadius()};
    ROS_DEBUG_STREAM(RED("Current radius : ") << WHITE(currentRadius_));
    return currentRadius_;
  };

  /**
   * @brief   Gets the target force value based on setup state.
   *
   * @return  Target force - includes tolerance during setup, otherwise just the
   *          needed force
   */
  [[nodiscard]] T getTargetForce() const {
    const T force_{m_needsSetup
                       ? m_config.force_needed + m_config.force_tolerance
                       : m_config.force_needed};
    ROS_DEBUG_STREAM(CYAN("Target force : ") << WHITE(force_));
    return force_;
  };

  /**
   * @brief   Calculates the difference between current force and target force.
   *
   * @param   force The current payload force
   * @return        The force difference if payload control is needed,
   *                otherwise 0
   */
  [[nodiscard]] T getTargetForceDiff(const T force) const {
    return m_payload.needsControl() ? force - getTargetForce() : 0;
  };

  /**
   * @brief   Gets the force tolerance threshold for control decisions.
   *
   * @param   minTolerance Minimum tolerance value (default: 1)
   * @return               The force tolerance - minimum if control is needed,
   *                       config value otherwise
   */
  [[nodiscard]] T getForceTolerance(const T minTolerance = T{1}) const {
    return m_payload.needsControl() ? minTolerance : m_config.force_tolerance;
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_MANIPULATORCONTROL_HPP_

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
  r2d2_state::LockStatusPair m_lockStatus{};
  r2d2_type::config::nozzle_t<T> m_config;

 protected:
  explicit ManipulatorConfig(const std::string& fileName = "nozzles")
      : IJsonConfigMap<r2d2_type::config::nozzle_t, T>{fileName} {};

 protected:
  void updateConfig() { m_config = this->getParams(m_nozzleType.key); };

 public:
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
  template <typename U>
  bool setLock(const U& value) {
    ROS_DEBUG_STREAM("setLock(val=" << WHITE(static_cast<int>(value)) << ")");
    m_lockStatus.updateType(value);
    if (m_lockStatus.key.empty()) {
      ROS_ERROR_STREAM("Got unknown lock status!");
      return false;
    }
    return true;
  };
  void resetMode() {
    ROS_DEBUG("Reset mode");
    m_workMode.updateType(r2d2_state::WorkMode::NONE);
  };
  void resetLock() {
    ROS_DEBUG("Reset lock");
    m_lockStatus.updateType(r2d2_state::LockStatus::LOCKED);
  };
};

template <typename T = double>
class ManipulatorControlHandler final : public ManipulatorConfig<T> {
 private:
  using ManipulatorConfig<T>::m_workMode;
  using ManipulatorConfig<T>::m_nozzleType;
  using ManipulatorConfig<T>::m_lockStatus;
  using ManipulatorConfig<T>::m_config;
  PipeHandler<T> m_pipe;
  PayloadHandler<T> m_payload;
  JointHandlerVector<T> m_joints;
  JointHandler<T>& m_shoulder{m_joints("shoulder")};
  JointHandler<T>& m_elbow{m_joints("elbow")};
  ros::Timer m_timer;
  volatile bool m_needsSetup{true};

 public:
  explicit ManipulatorControlHandler(ros::NodeHandle* node);
  ~ManipulatorControlHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~ManipulatorControlHandler()"));
    m_timer.stop();
  };

 private:
  void callbackManipulator(const ros::TimerEvent&);
  void processSetup(const T radius, const T force);
  void processControl(const T force);
  void processStop();
  // void processAngleControl(const T radius);
  // void processForceControl(const T force);

 protected:
  void publishResults() {
    ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
    m_joints.publish();
  };
  void checkSetup(const T force) {
    // ROS_DEBUG_STREAM(MAGENTA("\ncheckSetup()"));
    const bool needsAngleControl_{m_joints.needsControlAll()};
    const bool needsForceControl_{force < getTargetForce()};
    m_needsSetup = needsAngleControl_ && needsForceControl_;
    ROS_DEBUG_STREAM(CYAN("needsAngleControl_ = " << needsAngleControl_));
    ROS_DEBUG_STREAM(CYAN("needsForceControl_ = " << needsForceControl_));
    ROS_DEBUG_STREAM(CYAN("m_needsSetup = " << m_needsSetup));
    // ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::checkSetup()"));
  }
  void updateControlFlag(const T force) {
    m_payload.setControl(r2d2_math::abs(force) > getForceTolerance());
    ROS_DEBUG_STREAM(CYAN("needsForceControl = " << m_payload.needsControl()));
  };
  [[nodiscard]] int8_t getForceDiffSign(const T force) {
    const T forceDiff_{force - getTargetForce()};
    ROS_DEBUG_STREAM(BLUE("forceDiff = " << forceDiff_));
    updateControlFlag(forceDiff_);
    if (m_payload.needsControl()) return -r2d2_math::sign(forceDiff_);
    return 0;
  };

 public:
  [[nodiscard]] T getRadius() const {
    const T radius_{m_config.r0};
    ROS_DEBUG_STREAM("Base radius : " << WHITE(radius_));
    return radius_;
  };
  [[nodiscard]] T getCurrentRadius() const {
    const T currentRadius_{m_joints.getRadius() + getRadius()};
    ROS_DEBUG_STREAM(RED("Current radius : ") << WHITE(currentRadius_));
    return currentRadius_;
  };
  [[nodiscard]] T getTargetForce() const {
    const T force_{m_needsSetup
                       ? m_config.force_needed + m_config.force_tolerance
                       : m_config.force_needed};
    ROS_DEBUG_STREAM("Target force : " << WHITE(force_));
    return force_;
  };
  [[nodiscard]] T getForceTolerance(const T minTolerance = T{1}) const {
    return m_payload.needsControl() ? minTolerance : m_config.force_tolerance;
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_MANIPULATORCONTROL_HPP_

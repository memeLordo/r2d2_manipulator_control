#ifndef R2D2_MANIPULATOR_CONTROL_HPP
#define R2D2_MANIPULATOR_CONTROL_HPP

#include "JointHandler.hpp"
#include "PayloadHandler.hpp"
#include "PipeHandler.hpp"
#include "r2d2_utils_pkg/Json.hpp"
#include "r2d2_utils_pkg/Types.hpp"

template <typename T>
class ManipulatorConfig
    : private IJsonConfigMap<r2d2_type::config::manipulator_t, T> {
 protected:
  r2d2_state::WorkModePair m_workMode{};
  r2d2_state::NozzleTypePair m_nozzleType{};
  r2d2_state::LockStatusPair m_lockStatus{};
  r2d2_type::config::manipulator_t<T> m_config;

 protected:
  explicit ManipulatorConfig(const std::string& fileName = "manipulator")
      : IJsonConfigMap<r2d2_type::config::manipulator_t, T>{fileName} {};

 protected:
  void updateConfig() { m_config = this->getParams(m_nozzleType.key); };

 public:
  void resetMode() {
    ROS_DEBUG("Reset mode");
    m_workMode.type = r2d2_state::WorkMode::NONE;
  };
  void resetLock() {
    ROS_DEBUG("Reset lock");
    m_lockStatus.type = r2d2_state::LockStatus::NONE;
  };
  bool setMode(const T value) {
    ROS_DEBUG_STREAM("Set mode(value = " << WHITE(value) << ")");
    m_workMode.updateType(value);
    if (m_workMode.key.empty()) {
      ROS_ERROR_STREAM("Got unknown work mode!");
      return false;
    }
    return true;
  };
  bool setNozzle(const T value) {
    ROS_DEBUG_STREAM("Set nozzle(value = " << WHITE(value) << ")");
    m_nozzleType.updateType(value);
    if (m_nozzleType.key.empty()) {
      ROS_ERROR_STREAM("Got unknown nozzle type!");
      return false;
    }
    updateConfig();
    return true;
  };
  bool setLock(const T value) {
    ROS_DEBUG_STREAM("Set lock(value = " << WHITE(value) << ")");
    m_lockStatus.updateType(value);
    if (m_lockStatus.key.empty()) {
      ROS_ERROR_STREAM("Got unknown lock status!");
      return false;
    }
    return true;
  };
};

template <typename T = double>
class ManipulatorControlHandler : public ManipulatorConfig<T> {
 private:
  using ManipulatorConfig<T>::m_workMode;
  using ManipulatorConfig<T>::m_nozzleType;
  using ManipulatorConfig<T>::m_lockStatus;
  using ManipulatorConfig<T>::m_config;
  PipeHandler<T> m_pipe;
  PayloadHandler<T> m_payload;
  ShoulderHandler<T> m_shoulder;
  ElbowHandler<T> m_elbow;
  ros::Timer m_timer;
  std::mutex m_mutex;
  volatile bool m_needsSetup{true};

 public:
  explicit ManipulatorControlHandler(ros::NodeHandle* node);
  ~ManipulatorControlHandler() {
    ROS_DEBUG_STREAM(RED("~ManipulatorControlHandler()"));
    m_timer.stop();
  };

 private:
  void callbackManipulator(const ros::TimerEvent&);
  void processStop();
  void checkSetup(const T force);
  void processControl(const T radius, const T force);
  void processAngleControl(const T radius);
  void processForceControl(const T force);

 protected:
  bool needsForceControl(const T force) const {
    const bool needsForceControl_{r2d2_math::abs(force) > getForceTolerance()};
    ROS_DEBUG_STREAM(BLUE("needsForceControl_ = " << needsForceControl_));
    return needsForceControl_;
  };
  int8_t getForceDiff(const T force) const {
    const T forceDiff_{force - getTargetForce()};
    ROS_DEBUG_STREAM(BLUE("forceDiff_ = " << forceDiff_));
    if (needsForceControl(forceDiff_)) return -r2d2_math::sign(forceDiff_);
    return 0;
  };
  // TODO: перенести в JointMap
  void publishResults() {
    ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
    m_shoulder.publish();
    m_elbow.publish();
  };

 public:
  T getCurrentRadius() const {
    const T currentRadius_{m_shoulder.getRadius() + m_elbow.getRadius() +
                           getRadius()};
    ROS_DEBUG_STREAM(RED("Current radius : ") << WHITE(currentRadius_));
    return currentRadius_;
  };
  T getTargetForce(const T coeff = 1) const {
    T force_{coeff * static_cast<T>(m_config.force_needed)};
    ROS_DEBUG_STREAM("ManipulatorControl::getForce() : " << WHITE(force_));
    return force_;
  };
  T getForceTolerance() const {
    return static_cast<T>(m_config.force_tolerance);
  };
  T getRadius() const {
    const T radius_{m_config.r0};
    ROS_DEBUG_STREAM("ManipulatorControl::getRadius() : " << WHITE(radius_));
    return radius_;
  };
};
#endif  // R2D2_MANIPULATOR_CONTROL_HPP

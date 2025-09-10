#ifndef R2D2_MANIPULATOR_CONTROL_HPP
#define R2D2_MANIPULATOR_CONTROL_HPP

#include "JointHandler.hpp"
#include "PayloadHandler.hpp"
#include "PipeHandler.hpp"
#include "utils/IConfigJson.hpp"
#include "utils/Types.hpp"

template <typename T>
class ManipulatorConfig
    : public IConfigJsonTypes<r2d2_type::manipulator16_t<T>> {

protected:
  r2d2_state::WorkModePair m_workMode{};
  r2d2_state::NozzleTypePair m_nozzleType{};
  r2d2_state::LockStatusPair m_lockStatus{};
  r2d2_type::manipulator16_t<T> m_params;
  ManipulatorConfig()
      : IConfigJsonTypes<r2d2_type::manipulator16_t<T>>{"manipulator"} {};

public:
  void updateNozzleType() { m_params = this->getParams(m_nozzleType.key); };
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
    m_workMode.type = static_cast<r2d2_state::WorkMode>(value);
    m_workMode.toString();
    if (m_workMode.key.empty()) {
      ROS_ERROR_STREAM("Got unknown work mode");
      return false;
    }
    return true;
  };
  bool setNozzle(const T value) {
    ROS_DEBUG_STREAM("Set nozzle(value = " << WHITE(value) << ")");
    m_nozzleType.type = static_cast<r2d2_state::NozzleType>(value);
    m_nozzleType.toString();
    if (m_nozzleType.key.empty()) {
      ROS_ERROR_STREAM("Got unknown nozzle type");
      return false;
    }
    updateNozzleType();
    return true;
  };
  bool setLock(const T value) {
    ROS_DEBUG_STREAM("Set lock(value = " << WHITE(value) << ")");
    m_lockStatus.type = static_cast<r2d2_state::LockStatus>(value);
    m_lockStatus.toString();
    if (m_lockStatus.key.empty()) {
      ROS_ERROR_STREAM("Got unknown lock status");
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
  using ManipulatorConfig<T>::m_params;

  bool m_needsSetup{true};

  PayloadHandler<T> m_payload;
  PipeHandler<T> m_pipe;
  ElbowHandler<T> m_elbow;
  ShoulderHandler<T> m_shoulder;

  ros::Timer m_timer;

public:
  explicit ManipulatorControlHandler(ros::NodeHandle *node);

private:
  void callbackManipulator(const ros::TimerEvent &);
  void checkSetup(const T radius);
  void processStop(const T radius);
  void processControl(const T radius, const T force);
  void processAngleControl(const T radius);
  void processForceControl(const T force);

private:
  short checkForceDiff(const T force) const {
    const T forceDiff_{force - getTargetForce()};
    ROS_DEBUG_STREAM(BLUE("forceDiff_ = " << forceDiff_));
    const bool needsForceControl_{std::abs(forceDiff_) > getForceTolerance()};
    ROS_DEBUG_STREAM(BLUE("needsForceControl_ = " << needsForceControl_));
    if (needsForceControl_)
      return -r2d2_math::sign(forceDiff_);
    return 0;
  }
  T getCurrentRadius() const {
    const T currentRadius_{m_shoulder.getRadius() + m_elbow.getRadius() +
                           getRadius()};
    ROS_DEBUG_STREAM(RED("Current radius : ") << WHITE(currentRadius_));
    return currentRadius_;
  };
  void publishResults() {
    ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
    m_elbow.publish();
    m_shoulder.publish();
  };

public:
  T getTargetForce() const {
    const T force_{static_cast<T>(m_params.force_needed)};
    ROS_DEBUG_STREAM("ManipulatorControl::getForce() : " << WHITE(force_));
    return force_;
  };
  T getForceTolerance() const {
    return static_cast<T>(m_params.force_tolerance);
  };
  T getRadius() const {
    const T radius_{m_params.r0};
    ROS_DEBUG_STREAM("ManipulatorControl::getRadius() : " << WHITE(radius_));
    return radius_;
  };
};

#endif // R2D2_MANIPULATOR_CONTROL_HPP

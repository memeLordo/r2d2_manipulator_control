#ifndef R2D2_MANIPULATOR_CONTROL_HPP
#define R2D2_MANIPULATOR_CONTROL_HPP

#include "JointHandler.hpp"
#include "PayloadHandler.hpp"
#include "PipeHandler.hpp"
#include "utils/Types.hpp"

template <typename T = double> class ManipulatorControlHandler {

private:
  r2d2_state::WorkMode m_workMode{};
  r2d2_state::NozzleType m_nozzleType{};
  r2d2_state::LockStatus m_lockStatus{};

  r2d2_type::manipulator16_t<T> m_params{};

  bool m_needsSetup{true};

  PayloadHandler<T> m_payload;
  PipeHandler<T> m_pipe;
  ElbowHandler<T> m_elbow;
  ShoulderHandler<T> m_shoulder;

  ros::Timer m_timer;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);

private:
  void callbackManipulator(const ros::TimerEvent &);
  void checkSetup(const T radius);
  void processStop(const T radius);
  void processControl(const T radius, const T force);
  void processAngleControl(const T radius);
  void processForceControl(const T force);

private:
  short checkForceDiff(const T force) {
    const T forceDiff_ = force - getTargetForce();
    ROS_DEBUG_STREAM(BLUE("forceDiff_ = " << forceDiff_));
    const bool needsForceControl_{std::abs(forceDiff_) > getForceTolerance()};
    ROS_DEBUG_STREAM(BLUE("needsForceControl_ = " << needsForceControl_));
    if (needsForceControl_)
      return -r2d2_math::sign(forceDiff_);
    return 0;
  }
  T calcCurrentRadius() {
    return m_shoulder.getRadius() + m_elbow.getRadius() + getRadius();
  };
  void updateAngles() {
    m_elbow.updateAngle();
    m_shoulder.updateAngle();
  };
  void publishResults() {
    ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
    m_elbow.publish();
    m_shoulder.publish();
  };

public:
  void updateNozzleType();

public:
  void resetMode() {
    ROS_DEBUG("Reset mode");
    m_workMode = r2d2_state::WorkMode::NONE;
  };
  void resetNozzle() {
    ROS_DEBUG("Reset nozzle");
    m_nozzleType = r2d2_state::NozzleType::NONE;
  };
  void resetLock() {
    ROS_DEBUG("Reset lock");
    m_lockStatus = r2d2_state::LockStatus::NONE;
  };

  void setMode(r2d2_state::WorkMode value) {
    ROS_DEBUG_STREAM("Set mode(WorkMode)");
    m_workMode = value;
  };
  void setNozzle(r2d2_state::NozzleType value) {
    ROS_DEBUG_STREAM("Set nozzle(NozzleType)");
    m_nozzleType = value;
  };
  void setLock(r2d2_state::LockStatus value) {
    ROS_DEBUG_STREAM("Set lock(LockStatus)");
    m_lockStatus = value;
  };
  void setMode(T value) {
    ROS_DEBUG_STREAM("Set mode(value = " << WHITE(value) << ")");
    m_workMode = static_cast<r2d2_state::WorkMode>(value);
  };
  void setNozzle(T value) {
    ROS_DEBUG_STREAM("Set nozzle(value = " << WHITE(value) << ")");
    m_nozzleType = static_cast<r2d2_state::NozzleType>(value);
  };
  void setLock(T value) {
    ROS_DEBUG_STREAM("Set lock(value = " << WHITE(value) << ")");
    m_lockStatus = static_cast<r2d2_state::LockStatus>(value);
  };

  T getTargetForce() const {
    T force_ = static_cast<T>(m_params.force_needed);
    ROS_DEBUG_STREAM("ManipulatorControl::getForce() : " << WHITE(force_));
    return force_;
  };
  T getForceTolerance() const {
    return static_cast<T>(m_params.force_tolerance);
  };
  T getRadius() const {
    T radius_ = m_params.r0;
    ROS_DEBUG_STREAM("ManipulatorControl::getRadius() : " << WHITE(radius_));
    return radius_;
  };
};

#endif // R2D2_MANIPULATOR_CONTROL_HPP

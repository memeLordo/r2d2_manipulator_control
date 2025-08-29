#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "ElbowHandler.h"
#include "PayloadHandler.h"
#include "PipeHandler.h"
#include "ShoulderHandler.h"
#include "utils/Types.h"
#include <ros/node_handle.h>

template <typename T = double> class ManipulatorControlHandler {

private:
  r2d2_state::WorkMode m_workMode{};
  r2d2_state::NozzleType m_nozzleType{};
  r2d2_state::LockStatus m_lockStatus{};

  bool finishSetup{false};
  r2d2_types::manipulator16_t<T> m_params{};

  PayloadHandler<T> m_payload;
  PipeHandler<T> m_pipe;
  ElbowHandler<T> m_elbow;
  ShoulderHandler<T> m_shoulder;

  ros::Timer m_timer;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);

private:
  void callbackManipulator(const ros::TimerEvent &);
  T calcCurrentRadius();
  bool setup();
  void processControl();
  void processAngleControl();
  void processForceControl(const T currentForce, const T targetForce,
                           const T forceTreshold = 1e3);
  void publishResults();

public:
  void updateNozzleType();

public:
  void updateSetup() { finishSetup = true; }
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

  T getForce() const {
    auto force = static_cast<T>(m_params.force_needed);
    ROS_DEBUG_STREAM("ManipulatorControl::getForce() : " << WHITE(force));
    return force;
  };
  T getRadius() const {
    auto radius = m_params.r0;
    ROS_DEBUG_STREAM("ManipulatorControl::getRadius() : " << WHITE(radius));
    return radius;
  };
};

#endif // MANIPULATOR_CONTROL_H

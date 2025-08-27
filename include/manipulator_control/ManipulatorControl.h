#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "ElbowHandler.h"
#include "PayloadHandler.h"
#include "PipeHandler.h"
#include "ShoulderHandler.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ManipulatorControlHandler {

public:
  enum class WorkMode : uint8_t { NONE = 0, MANUAL, AUTO } m_workMode{};
  enum class NozzleType : uint8_t { NONE = 0, BRUSH, EMA } m_nozzleType{};
  enum class LockStatus : uint8_t { NONE = 0, LOCKED, UNLOCKED } m_lockStatus{};

private:
  struct manipulator_t {
    int16_t force_needed{};
    T r0{};
  } m_params{};

  PayloadHandler<T> m_payload;
  PipeHandler<T> m_pipe;
  ElbowHandler<T> m_elbow;
  ShoulderHandler<T> m_shoulder;

  ros::Timer m_timer;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);

private:
  void callbackManipulator(const ros::TimerEvent &);
  void setup();
  T calcRadius();
  void processAngleControl();
  void processForceControl();
  void publishResults();
  void updateJointState();
  void publishJointState();

public:
  void updateNozzleType();

public:
  void resetMode() {
    ROS_DEBUG("Reset mode");
    m_workMode = WorkMode::NONE;
  };
  void resetNozzle() {
    ROS_DEBUG("Reset nozzle");
    m_nozzleType = NozzleType::NONE;
  };
  void resetLock() {
    ROS_DEBUG("Reset lock");
    m_lockStatus = LockStatus::NONE;
  };

  void setMode(WorkMode value) {
    ROS_DEBUG_STREAM("Set mode(WorkMode)");
    m_workMode = value;
  };
  void setNozzle(NozzleType value) {
    ROS_DEBUG_STREAM("Set nozzle(NozzleType)");
    m_nozzleType = value;
  };
  void setLock(LockStatus value) {
    ROS_DEBUG_STREAM("Set lock(LockStatus)");
    m_lockStatus = value;
  };
  void setMode(T value) {
    ROS_DEBUG_STREAM("Set mode(" << WHITE(value) << GREEN(")"));
    m_workMode = static_cast<WorkMode>(value);
  };
  void setNozzle(T value) {
    ROS_DEBUG_STREAM("Set nozzle(" << WHITE(value) << GREEN(")"));
    m_nozzleType = static_cast<NozzleType>(value);
  };
  void setLock(T value) {
    ROS_DEBUG_STREAM("Set lock(" << WHITE(value) << GREEN(")"));
    m_lockStatus = static_cast<LockStatus>(value);
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

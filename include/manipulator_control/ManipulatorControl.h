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

  void updateNozzleType();

public:
  void resetMode() { m_workMode = WorkMode::NONE; };
  void resetNozzle() { m_nozzleType = NozzleType::NONE; };
  void resetLock() { m_lockStatus = LockStatus::NONE; };

  void setMode(WorkMode value) { m_workMode = value; };
  void setNozzle(NozzleType value) { m_nozzleType = value; };
  void setLock(LockStatus value) { m_lockStatus = value; };
  void setMode(T value) { m_workMode = static_cast<WorkMode>(value); };
  void setNozzle(T value) { m_nozzleType = static_cast<NozzleType>(value); };
  void setLock(T value) { m_lockStatus = static_cast<LockStatus>(value); };

  T getForce() const { return static_cast<T>(m_params.force_needed); };
  T getRadius() const { return m_params.r0; };
};

#endif // MANIPULATOR_CONTROL_H

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
  enum class WorkMode : uint8_t { NONE = 0, MANUAL, AUTO } mode{};
  enum class NozzleType : uint8_t { NONE = 0, BRUSH, EMA } nozzle{};
  enum class LockStatus : uint8_t { NONE = 0, LOCKED, UNLOCKED } status{};

private:
  struct manipulator_t {
    int16_t force_needed{};
    T r0{};
  } params{};

  void update_nozzle_type() {
    switch (nozzle) {
    case NozzleType::BRUSH:
      params = manipulator_t{100, 347.0};
      return;
    case NozzleType::EMA:
      params = manipulator_t{150, 331.0};
      return;
    }
  }

  PayloadHandler<T> payload;
  PipeHandler<T> pipe;
  ElbowHandler<T> elbow;
  ShoulderHandler<T> shoulder;

  ros::Timer timer;

  T calc_radius();
  void process_angle_control();
  void process_force_control();
  void publish_results();
  void update_joint_state();
  void publish_joint_state();
  void setup();
  void callback_manipulator(const ros::TimerEvent &);

public:
  void reset_mode() { mode = WorkMode::NONE; };
  void reset_nozzle() { nozzle = NozzleType::NONE; };
  void reset_lock() { status = LockStatus::NONE; };

  void set_mode(WorkMode value) { mode = value; };
  void set_nozzle(NozzleType value) { nozzle = value; };
  void set_lock(LockStatus value) { status = value; };
  void set_mode(T value) { mode = static_cast<WorkMode>(value); };
  void set_nozzle(T value) { nozzle = static_cast<NozzleType>(value); };
  void set_lock(T value) { status = static_cast<LockStatus>(value); };

  T get_force() const { return static_cast<T>(params.force_needed); };
  T get_radius() const { return params.r0; };

  ManipulatorControlHandler(ros::NodeHandle *node);
};

#endif // MANIPULATOR_CONTROL_H

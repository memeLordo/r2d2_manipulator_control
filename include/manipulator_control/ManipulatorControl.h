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
  enum class WorkMode : uint8_t { NONE = 0, MANUAL, AUTO } work_mode{};
  enum class NozzleType : uint8_t { NONE = 0, BRUSH, EMA } nozzle_type{};
  enum class LockStatus : uint8_t { NONE = 0, LOCKED, UNLOCKED } lock_status{};

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

public:
  ManipulatorControlHandler(ros::NodeHandle *node);

private:
  void callback_manipulator(const ros::TimerEvent &);
  void setup();
  T calc_radius();
  void process_angle_control();
  void process_force_control();
  void publish_results();
  void update_joint_state();
  void publish_joint_state();

  void update_nozzle_type() {
    switch (nozzle_type) {
    case NozzleType::BRUSH:
      params = manipulator_t{100, 347.0};
      return;
    case NozzleType::EMA:
      params = manipulator_t{150, 331.0};
      return;
    }
  };

public:
  void reset_mode() { work_mode = WorkMode::NONE; };
  void reset_nozzle() { nozzle_type = NozzleType::NONE; };
  void reset_lock() { lock_status = LockStatus::NONE; };

  void set_mode(WorkMode value) { work_mode = value; };
  void set_nozzle(NozzleType value) { nozzle_type = value; };
  void set_lock(LockStatus value) { lock_status = value; };
  void set_mode(T value) { work_mode = static_cast<WorkMode>(value); };
  void set_nozzle(T value) { nozzle_type = static_cast<NozzleType>(value); };
  void set_lock(T value) { lock_status = static_cast<LockStatus>(value); };

  T get_force() const { return static_cast<T>(params.force_needed); };
  T get_radius() const { return params.r0; };
};

#endif // MANIPULATOR_CONTROL_H

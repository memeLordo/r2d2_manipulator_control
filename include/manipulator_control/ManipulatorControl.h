#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "ElbowHandler.h"
#include "PayloadHandler.h"
#include "PipeHandler.h"
#include "ShoulderHandler.h"
#include <cstdint>
#include <ros/node_handle.h>

template <typename T = double> class ManipulatorControlHandler {

private:
  enum WorkMode : uint8_t { MANUAL = 0, AUTO } mode{MANUAL};         // ???
  enum NozzleType : uint8_t { NONE = 0, BRUSH, EMA } nozzle{NONE};   // setup
  enum LockStatus : uint8_t { LOCKED = 0, UNLOCKED } status{LOCKED}; // callback
  struct manipulator_t {
    int16_t force_needed{};
    T r0{};
  } params;

  constexpr manipulator_t get_nozzle_type() {
    switch (nozzle) {
    case BRUSH:
      return {100, 347.0};
    case EMA:
      return {150, 331.0};
    default:
      return {0, 0.0};
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
  void update_all();
  void publish_all();
  void setup();
  void callback_manipulator(const ros::TimerEvent &);

  void set_mode(T value) { mode = static_cast<WorkMode>(value); };
  void set_nozzle(T value) { nozzle = static_cast<NozzleType>(value); };
  void set_lock(T value) { status = static_cast<LockStatus>(value); };
  T get_force() const { return static_cast<T>(params.force_needed); };
  T get_radius() const { return params.r0; };

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
};

#endif // MANIPULATOR_CONTROL_H

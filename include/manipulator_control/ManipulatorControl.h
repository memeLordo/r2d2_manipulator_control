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
  enum NozzleType : uint8_t { NONE = 0, BRUSH, EMA } nozzle{NONE};
  enum LockStatus : uint8_t { LOCKED = 0, UNLOCKED } status{LOCKED};
  enum WorkMode : uint8_t { MANUAL = 0, AUTO } mode{MANUAL};

  struct manipulator_t {
    int16_t force_needed{};
    T r0{};
  } params;
  manipulator_t callback_params;

  constexpr manipulator_t get_nozzle_type(NozzleType nozzle) {
    switch (nozzle) {
    case BRUSH:
      return {100, 347.0};
    case EMA:
      return {150, 331.0};
    default:
      return {0, 0.0};
    }
  }

  PipeHandler<T> pipe;
  ElbowHandler<T> elbow;
  ShoulderHandler<T> shoulder;
  PayloadHandler<T> payload;

  ros::Timer timer;
  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
  void setup();
  void callback_manipulator(const ros::TimerEvent &);
  void set_nozzle(T value) { nozzle = static_cast<NozzleType>(value); };
  void set_lock(T value) { status = static_cast<LockStatus>(value); };
  void set_mode(T value) { mode = static_cast<WorkMode>(value); };
  void update();
  void update_all();
  void publish_all();
  auto get_nozzle() const { return nozzle; };
  auto get_lock() const { return status; };
  auto get_mode() const { return mode; };
  auto calc_radius();
  T get_force() const { return static_cast<T>(params.force_needed); };
  T get_radius() const { return params.r0; };
};

#endif // MANIPULATOR_CONTROL_H

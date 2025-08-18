#ifndef MANIPULATOR_CONTROL_H
#define MANIPULATOR_CONTROL_H

#include "ElbowHandler.h"
#include "PayloadHandler.h"
#include "ShoulderHandler.h"
#include <cstdint>
#include <ros/ros.h>

class ManipulatorControlHandler {
private:
  enum NozzleType : uint8_t { NONE = 0, BRUSH, EMA } nozzle{NONE};
  enum LockStatus : uint8_t { LOCKED = 0, UNLOCKED } status{LOCKED};
  enum WorkMode : uint8_t { MANUAL = 0, AUTO } mode{MANUAL};

  struct manipulator_t {
    int16_t force_needed{};
    double r0{};
  } params;
  manipulator_t callback_params;

  ElbowHandler elbow;
  ShoulderHandler shoulder;
  PayloadHandler payload;

  ros::Subscriber subscriber;
  ros::Publisher publisher;

public:
  ManipulatorControlHandler(ros::NodeHandle *node);
  void setup();
  void callback_manipulator();
  template <typename T> void set_nozzle(T value) {
    nozzle = static_cast<NozzleType>(value);
  };
  template <typename T> void set_lock(T value) {
    status = static_cast<LockStatus>(value);
  };
  template <typename T> void set_mode(T value) {
    mode = static_cast<WorkMode>(value);
  };
  void update();
  void update_all();
  void publish_all();
  auto get_nozzle() const { return nozzle; };
  auto get_status() const { return status; };
  auto get_mode() const { return mode; };
  template <typename T = double> T get_force() const {
    return static_cast<T>(params.force_needed);
  };
  template <typename T = double> T get_radius() const {
    return static_cast<T>(params.r0);
  };
  auto calc_radius();
};

#endif // MANIPULATOR_CONTROL_H

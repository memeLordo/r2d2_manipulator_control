#include "ManipulatorService.h"
// #include <string>
//
// const std::string SERVICE_MODE = "set_mode";
// const std::string SERVICE_NOZZLE = "set_nozzle";
// const std::string SERVICE_STATUS = "set_status";
//
ManipulatorServiceHandler::ManipulatorServiceHandler(ros::NodeHandle *node) {
  mode_service_ = node->advertiseService(
      "set_mode", &ManipulatorServiceHandler::callback_mode_service, this);
  nozzle_service_ = node->advertiseService(
      "set_nozzle", &ManipulatorServiceHandler::callback_nozzle_service, this);
  status_service_ = node->advertiseService(
      "set_status", &ManipulatorServiceHandler::callback_status_service, this);
  ROS_INFO("ManipulatorServiceHandler::ManipulatorServiceHandler");
}

bool ManipulatorServiceHandler::callback_mode_service(
    manipulator_control::SetWorkMode::Request &req,
    manipulator_control::SetWorkMode::Response &res) {
  if (req.mode > 1) {
    res.success = false;
    res.message = "Invalid mode value. Must be 0 (MANUAL) or 1 (AUTO)";
    return true;
  }

  auto mode_ = req.mode;
  res.success = true;
  res.message = (req.mode == 0) ? "Mode set to MANUAL" : "Mode set to AUTO";

  ROS_INFO("Mode updated to %s", (mode_ == 0) ? "MANUAL" : "AUTO");
  return true;
}

bool ManipulatorServiceHandler::callback_nozzle_service(
    manipulator_control::SetNozzleType::Request &req,
    manipulator_control::SetNozzleType::Response &res) {
  if (req.nozzle_type > 2) {
    res.success = false;
    res.message =
        "Invalid nozzle type. Must be 0 (NONE), 1 (BRUSH), or 2 (EMA)";
    return true;
  }

  auto nozzle_ = req.nozzle_type;
  res.success = true;

  std::string nozzle_names[] = {"NONE", "BRUSH", "EMA"};
  res.message = "Nozzle type set to " + nozzle_names[req.nozzle_type];

  ROS_INFO("Nozzle type updated to %s", nozzle_names[nozzle_].c_str());
  return res.success = true;
}

bool ManipulatorServiceHandler::callback_status_service(
    manipulator_control::SetLockStatus::Request &req,
    manipulator_control::SetLockStatus::Response &res) {
  if (req.lock_status > 1) {
    res.success = false;
    res.message = "Invalid lock status. Must be 0 (LOCKED) or 1 (UNLOCKED)";
    return true;
  }

  auto lock_status_ = req.lock_status;
  res.success = true;
  res.message = (req.lock_status == 0) ? "Status set to LOCKED"
                                       : "Status set to UNLOCKED";

  ROS_INFO("Lock status updated to %s",
           (lock_status_ == 0) ? "LOCKED" : "UNLOCKED");
  return true;
}

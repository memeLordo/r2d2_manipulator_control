#include "ManipulatorServiceHandler.h"

constexpr std::string SERVICE_MODE = "set_mode";
constexpr std::string SERVICE_NOZZLE = "set_nozzle";
constexpr std::string SERVICE_STATUS = "set_status";

ManipulatorServiceHandler::Manipulator ServiceHandler(NodeHandle *node) {
  mode_serer = node->advertiseService(
      "set_mode", &ManipulatorServiceHandler::callback_mode, this);
  nozzle_server = node->advertiseService(
      "set_nozzle", &ManipulatorServiceHandler::callback_nozzle, this);
  status_server = node->advertiseService(
      "set_status", &ManipulatorServiceHandler::callback_status, this);
}

bool ManipulatorServiceHandler::setModeCallback(
    manipulator_control::SetMode::Response &res) {
  if (req.mode > 1) {
    res.success = false;
    res.message = "Invalid mode value. Must be 0 (MANUAL) or 1 (AUTO)";
    return true;
  }

  mode_ = req.mode;
  res.success = true;
  res.message = (req.mode == 0) ? "Mode set to MANUAL" : "Mode set to AUTO";

  ROS_INFO("Mode updated to %s", (mode_ == 0) ? "MANUAL" : "AUTO");
  return true;
}

bool ManipulatorServiceHandler::callback_nozzle_service(
    manipulator_control::SetNozzle::Request &req,
    manipulator_control::SetNozzle::Response &res) {
  if (req.nozzle_type > 2) {
    res.success = false;
    res.message =
        "Invalid nozzle type. Must be 0 (NONE), 1 (BRUSH), or 2 (EMA)";
    return true;
  }

  nozzle_ = req.nozzle_type;
  res.success = true;

  std::string nozzle_names[] = {"NONE", "BRUSH", "EMA"};
  res.message = "Nozzle type set to " + nozzle_names[req.nozzle_type];

  ROS_INFO("Nozzle type updated to %s", nozzle_names[nozzle_].c_str());
  return true;
}

bool ManipulatorServiceHandler::callback_status_service(
    manipulator_control::SetStatus::Request &req,
    manipulator_control::SetStatus::Response &res) {
  if (req.lock_status > 1) {
    res.success = false;
    res.message = "Invalid lock status. Must be 0 (LOCKED) or 1 (UNLOCKED)";
    return true;
  }

  lock_status_ = req.lock_status;
  res.success = true;
  res.message = (req.lock_status == 0) ? "Status set to LOCKED"
                                       : "Status set to UNLOCKED";

  ROS_INFO("Lock status updated to %s",
           (lock_status_ == 0) ? "LOCKED" : "UNLOCKED");
  return true;
}

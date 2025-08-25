#include "ManipulatorService.h"
#include "ManipulatorControl.h"

using WorkMode = ManipulatorControlHandler<>::WorkMode;
using NozzleType = ManipulatorControlHandler<>::NozzleType;
using LockStatus = ManipulatorControlHandler<>::LockStatus;
// #include <string>
//
// const std::string SERVICE_MODE = "set_mode";
// const std::string SERVICE_NOZZLE = "set_nozzle";
// const std::string SERVICE_STATUS = "set_status";
//
ManipulatorServiceHandler::ManipulatorServiceHandler(ros::NodeHandle *node) {
  mode_service_ = node->advertiseService(
      "/set_mode", &ManipulatorServiceHandler::callback_mode_service, this);
  nozzle_service_ = node->advertiseService(
      "/set_nozzle", &ManipulatorServiceHandler::callback_nozzle_service, this);
  status_service_ = node->advertiseService(
      "/set_status", &ManipulatorServiceHandler::callback_status_service, this);
}

bool ManipulatorServiceHandler::callback_mode_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  ROS_INFO("callback_mode_service::got request");
  auto mode_ = static_cast<WorkMode>(req.work_mode);
  switch (mode_) {
  case WorkMode::MANUAL:
  case WorkMode::AUTO:
    // manipulator_control.set_mode(mode_);
    res.success = true;
  default:
    res.success = false;
  }
  return true;
}
bool ManipulatorServiceHandler::callback_nozzle_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  ROS_INFO("callback_nozzle_service::got request");
  auto nozzle_ = static_cast<NozzleType>(req.nozzle_type);
  switch (nozzle_) {
  case NozzleType::BRUSH:
  case NozzleType::EMA:
    // manipulator_control.set_nozzle(nozzle_);
    res.success = true;
  default:
    res.success = false;
  }
  return true;
}
bool ManipulatorServiceHandler::callback_status_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  ROS_INFO("callback_status_service::got request");
  auto status_ = static_cast<LockStatus>(req.lock_status);
  switch (status_) {
  case LockStatus::LOCKED:
  case LockStatus::UNLOCKED:
    // manipulator_control.set_lock(status_);
    res.success = true;
  default:
    res.success = false;
  }
  return true;
}

// template class ManipulatorServiceHandler<>;

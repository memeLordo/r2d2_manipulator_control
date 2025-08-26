#include "ManipulatorService.h"

using WorkMode = ManipulatorControlHandler<>::WorkMode;
using NozzleType = ManipulatorControlHandler<>::NozzleType;
using LockStatus = ManipulatorControlHandler<>::LockStatus;

constexpr const char *MANIPULATOR_SERVICE_MODE = "/manipulator_command";

ManipulatorServiceHandler::ManipulatorServiceHandler(
    ros::NodeHandle *node, ManipulatorControlHandler<> &manipulator_controlRef)
    : manipulator_control(manipulator_controlRef) {
  manipulator_service = node->advertiseService(
      MANIPULATOR_SERVICE_MODE, &ManipulatorServiceHandler::callback_service,
      this);
}
bool ManipulatorServiceHandler::callback_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  res.success = true;
  return callback_mode_service(req, res) && callback_nozzle_service(req, res) &&
         callback_status_service(req, res);
}

bool ManipulatorServiceHandler::callback_mode_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  ROS_INFO("callback_mode_service::got request, work_mode: %d", req.work_mode);
  auto work_mode_ = static_cast<WorkMode>(req.work_mode);
  switch (work_mode_) {
  case WorkMode::MANUAL:
  case WorkMode::AUTO:
    manipulator_control.set_mode(work_mode_);
    break;
  default:
    ROS_ERROR("Got unknown work mode");
    res.success &= false;
  }
  return true;
}
bool ManipulatorServiceHandler::callback_nozzle_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  ROS_INFO("callback_nozzle_service::got request, nozzle_type: %d",
           req.nozzle_type);
  auto nozzle_type_ = static_cast<NozzleType>(req.nozzle_type);
  switch (nozzle_type_) {
  case NozzleType::BRUSH:
  case NozzleType::EMA:
    manipulator_control.set_nozzle(nozzle_type_);
    break;
  default:
    ROS_ERROR("Got unknown nozzle type");
    res.success &= false;
  }
  return true;
}
bool ManipulatorServiceHandler::callback_status_service(
    manipulator_control::ManipulatorCommand::Request &req,
    manipulator_control::ManipulatorCommand::Response &res) {
  ROS_INFO("callback_status_service::got request %d", req.lock_status);
  auto lock_status_ = static_cast<LockStatus>(req.lock_status);
  switch (lock_status_) {
  case LockStatus::LOCKED:
  case LockStatus::UNLOCKED:
    manipulator_control.set_lock(lock_status_);
    break;
  default:
    ROS_ERROR("Got unknown lock status");
    res.success &= false;
  }
  return true;
}

#include "ManipulatorService.hpp"
#include <ros/console.h>

using namespace r2d2_state;

ManipulatorServiceHandler::ManipulatorServiceHandler(
    ros::NodeHandle *node, ManipulatorControlHandler<> &manipulator_controlRef)
    : m_serviceNode{"/manipulator_command"},
      m_manipulatorControl(manipulator_controlRef) {
  m_manipulatorService = node->advertiseService(
      m_serviceNode, &ManipulatorServiceHandler::callbackService, this);
}
bool ManipulatorServiceHandler::callbackService(
    r2d2_msg_pkg::ManipulatorCommand::Request &req,
    r2d2_msg_pkg::ManipulatorCommand::Response &res) {
  res.success = true;
  return callbackModeService(req, res) && callbackNozzleService(req, res) &&
         callbackStatusService(req, res);
}

bool ManipulatorServiceHandler::callbackModeService(
    r2d2_msg_pkg::ManipulatorCommand::Request &req,
    r2d2_msg_pkg::ManipulatorCommand::Response &res) {
  ROS_DEBUG_STREAM(
      "callbackModeService::got request, work_mode: " << req.work_mode);
  auto work_mode_ = static_cast<WorkMode>(req.work_mode);
  switch (work_mode_) {
  case WorkMode::MANUAL:
  case WorkMode::AUTO:
    m_manipulatorControl.setMode(work_mode_);
    break;
  default:
    ROS_ERROR("Got unknown work mode");
    res.success &= false;
  }
  return true;
}
bool ManipulatorServiceHandler::callbackNozzleService(
    r2d2_msg_pkg::ManipulatorCommand::Request &req,
    r2d2_msg_pkg::ManipulatorCommand::Response &res) {
  ROS_DEBUG_STREAM(
      "callback_nozzle_service::got request, nozzle_type: " << req.nozzle_type);
  auto nozzle_type_ = static_cast<NozzleType>(req.nozzle_type);
  switch (nozzle_type_) {
  case NozzleType::BRUSH:
  case NozzleType::EMA:
    m_manipulatorControl.setNozzle(nozzle_type_);
    m_manipulatorControl.updateNozzleType();
    break;
  default:
    ROS_ERROR("Got unknown nozzle type");
    res.success &= false;
  }
  return true;
}
bool ManipulatorServiceHandler::callbackStatusService(
    r2d2_msg_pkg::ManipulatorCommand::Request &req,
    r2d2_msg_pkg::ManipulatorCommand::Response &res) {
  ROS_DEBUG_STREAM(
      "callback_status_service::got request, lock_status: " << req.lock_status);
  auto lock_status_ = static_cast<LockStatus>(req.lock_status);
  switch (lock_status_) {
  case LockStatus::LOCKED:
  case LockStatus::UNLOCKED:
    m_manipulatorControl.setLock(lock_status_);
    break;
  default:
    ROS_ERROR("Got unknown lock status");
    res.success &= false;
  }
  return true;
}

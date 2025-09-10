#include "ManipulatorService.hpp"

using namespace r2d2_state;
using namespace r2d2_msg_pkg;

bool ManipulatorServiceHandler::callbackService(
    ManipulatorCommand::Request &req, ManipulatorCommand::Response &res) {
  res.success = true;
  return callbackModeService(req, res) && callbackNozzleService(req, res) &&
         callbackStatusService(req, res);
}

bool ManipulatorServiceHandler::callbackModeService(
    ManipulatorCommand::Request &req, ManipulatorCommand::Response &res) {
  ROS_DEBUG_STREAM(
      "callbackModeService::got request, work_mode: " << req.work_mode);
  if (!m_manipulatorControl->setMode(req.work_mode))
    res.success &= false;
  return true;
}
bool ManipulatorServiceHandler::callbackNozzleService(
    ManipulatorCommand::Request &req, ManipulatorCommand::Response &res) {
  ROS_DEBUG_STREAM(
      "callback_nozzle_service::got request, nozzle_type: " << req.nozzle_type);
  if (!m_manipulatorControl->setNozzle(req.nozzle_type))
    res.success &= false;
  return true;
}
bool ManipulatorServiceHandler::callbackStatusService(
    ManipulatorCommand::Request &req, ManipulatorCommand::Response &res) {
  ROS_DEBUG_STREAM(
      "callback_status_service::got request, lock_status: " << req.lock_status);
  if (!m_manipulatorControl->setLock(req.lock_status))
    res.success &= false;
  return true;
}

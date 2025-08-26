#ifndef R2D2_MANIPULATOR_SERVICE_H
#define R2D2_MANIPULATOR_SERVICE_H

#include "ManipulatorControl.hpp"
#include "r2d2_msg_pkg/ManipulatorCommand.h"

class ManipulatorServiceHandler {
 private:
  static constexpr const char *s_name = "ManipulatorService";

 private:
  const std::string m_serviceTopic;
  ManipulatorControlHandler<> *m_manipulatorControl;
  ros::ServiceServer m_manipulatorService;

 public:
  explicit ManipulatorServiceHandler(
      ros::NodeHandle *node, ManipulatorControlHandler<> *manipulatorControlPtr)
      : m_serviceTopic{"/manipulator_command"},
        m_manipulatorControl{manipulatorControlPtr} {
    m_manipulatorService = node->advertiseService(
        m_serviceTopic, &ManipulatorServiceHandler::callbackService, this);
  };

 private:
  bool callbackService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                       r2d2_msg_pkg::ManipulatorCommand::Response &res) {
    res.success = true;
    return callbackModeService(req, res) && callbackNozzleService(req, res) &&
           callbackStatusService(req, res);
  };
  bool callbackModeService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                           r2d2_msg_pkg::ManipulatorCommand::Response &res) {
    ROS_DEBUG_STREAM(
        "callbackModeService::got request, work_mode: " << req.work_mode);
    if (!m_manipulatorControl->setMode(req.work_mode)) res.success &= false;
    return true;
  };
  bool callbackNozzleService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                             r2d2_msg_pkg::ManipulatorCommand::Response &res) {
    ROS_DEBUG_STREAM("callback_nozzle_service::got request, nozzle_type: "
                     << req.nozzle_type);
    if (!m_manipulatorControl->setNozzle(req.nozzle_type)) res.success &= false;
    return true;
  };
  bool callbackStatusService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                             r2d2_msg_pkg::ManipulatorCommand::Response &res) {
    ROS_DEBUG_STREAM("callback_status_service::got request, lock_status: "
                     << req.lock_status);
    if (!m_manipulatorControl->setLock(req.lock_status)) res.success &= false;
    return true;
  };
};
#endif  // R2D2_MANIPULATOR_SERVICE_H

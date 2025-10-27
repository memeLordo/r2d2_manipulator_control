#ifndef R2D2_MANIPULATOR_SERVICE_H
#define R2D2_MANIPULATOR_SERVICE_H

#include "ManipulatorControl.hpp"
#include "r2d2_msg_pkg/ManipulatorCommand.h"

class ManipulatorServiceHandler final {
 private:
  const std::string m_serviceTopic;
  ManipulatorControlHandler<>& m_manipulatorControl;
  ros::ServiceServer m_manipulatorService;

 public:
  explicit ManipulatorServiceHandler(
      ros::NodeHandle* node, ManipulatorControlHandler<>& manipulatorControlRef)
      : m_serviceTopic{"/manipulator_command"},
        m_manipulatorControl{manipulatorControlRef} {
    m_manipulatorService = node->advertiseService(
        m_serviceTopic, &ManipulatorServiceHandler::callbackService, this);
  };
  ~ManipulatorServiceHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~ManipulatorServiceHandler()"));
    m_manipulatorService.shutdown();
  };

 private:
  [[nodiscard]] bool callbackService(
      r2d2_msg_pkg::ManipulatorCommand::Request& req,
      r2d2_msg_pkg::ManipulatorCommand::Response& res) {
    res.success = true;
    return callbackModeService(req, res) && callbackNozzleService(req, res);
  };

  [[nodiscard]] bool callbackModeService(
      r2d2_msg_pkg::ManipulatorCommand::Request& req,
      r2d2_msg_pkg::ManipulatorCommand::Response& res) {
    ROS_DEBUG_STREAM(
        "callbackModeService::got request, work_mode: " << req.work_mode);
    if (!m_manipulatorControl.setMode(req.work_mode)) res.success &= false;
    return true;
  };

  [[nodiscard]] bool callbackNozzleService(
      r2d2_msg_pkg::ManipulatorCommand::Request& req,
      r2d2_msg_pkg::ManipulatorCommand::Response& res) {
    ROS_DEBUG_STREAM(
        "callbackNozzleService::got request, nozzle_type: " << req.nozzle_type);
    if (!m_manipulatorControl.setNozzle(req.nozzle_type)) res.success &= false;
    return true;
  };
};
#endif  // R2D2_MANIPULATOR_SERVICE_H

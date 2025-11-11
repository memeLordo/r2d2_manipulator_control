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
  /**
   * @brief Constructs a ManipulatorServiceHandler and advertises the
   * manipulator command service.
   * @param node Pointer to the ROS node handle
   * @param manipulatorControlRef Reference to the manipulator control handler
   * @details Sets up a ROS service server for receiving manipulator commands.
   */
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
  /**
   * @brief Main service callback that handles manipulator command requests.
   * @param req The service request containing work mode and nozzle type
   * @param res The service response indicating success/failure
   * @return True if the service call was processed successfully
   * @details Processes both mode and nozzle type changes from the request.
   */
  [[nodiscard]] bool callbackService(
      r2d2_msg_pkg::ManipulatorCommand::Request& req,
      r2d2_msg_pkg::ManipulatorCommand::Response& res) {
    res.success = true;
    return callbackModeService(req, res) && callbackNozzleService(req, res);
  };

  /**
   * @brief Handles work mode change requests from the service.
   * @param req The service request containing the work mode
   * @param res The service response (success flag is updated)
   * @return True if the callback was processed
   * @details Sets the work mode in the manipulator control handler.
   */
  [[nodiscard]] bool callbackModeService(
      r2d2_msg_pkg::ManipulatorCommand::Request& req,
      r2d2_msg_pkg::ManipulatorCommand::Response& res) {
    ROS_DEBUG_STREAM(
        "callbackModeService::got request, work_mode: " << req.work_mode);
    if (!m_manipulatorControl.setMode(req.work_mode)) res.success &= false;
    return true;
  };

  /**
   * @brief Handles nozzle type change requests from the service.
   * @param req The service request containing the nozzle type
   * @param res The service response (success flag is updated)
   * @return True if the callback was processed
   * @details Sets the nozzle type in the manipulator control handler.
   */
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

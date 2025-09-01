#ifndef MANIPULATOR_SERVICE_H
#define MANIPULATOR_SERVICE_H
#include "ManipulatorControl.h"
#include "r2d2_msg_pkg/ManipulatorCommand.h"
#include "ros/node_handle.h"

class ManipulatorServiceHandler {
 private:
  ManipulatorControlHandler<>& m_manipulatorControl;

  ros::ServiceServer m_manipulatorService;

 public:
  ManipulatorServiceHandler(ros::NodeHandle* node,
                            ManipulatorControlHandler<>&);

 private:
  bool callbackService(r2d2_msg_pkg::ManipulatorCommand::Request& req,
                       r2d2_msg_pkg::ManipulatorCommand::Response& res);
  bool callbackModeService(r2d2_msg_pkg::ManipulatorCommand::Request& req,
                           r2d2_msg_pkg::ManipulatorCommand::Response& res);
  bool callbackNozzleService(r2d2_msg_pkg::ManipulatorCommand::Request& req,
                             r2d2_msg_pkg::ManipulatorCommand::Response& res);
  bool callbackStatusService(r2d2_msg_pkg::ManipulatorCommand::Request& req,
                             r2d2_msg_pkg::ManipulatorCommand::Response& res);
};
#endif  // MANIPULATOR_SERVICE_H

#ifndef MANIPULATOR_SERVICE_H
#define MANIPULATOR_SERVICE_H
#include "ManipulatorControl.h"
#include "manipulator_control/ManipulatorCommand.h"

#include "ros/node_handle.h"
class ManipulatorServiceHandler {
private:
  ManipulatorControlHandler<> &manipulator_control;

  ros::ServiceServer manipulator_service;

public:
  ManipulatorServiceHandler(ros::NodeHandle *node,
                            ManipulatorControlHandler<> &);

private:
  bool callback_service(manipulator_control::ManipulatorCommand::Request &req,
                        manipulator_control::ManipulatorCommand::Response &res);
  bool
  callback_mode_service(manipulator_control::ManipulatorCommand::Request &req,
                        manipulator_control::ManipulatorCommand::Response &res);
  bool callback_nozzle_service(
      manipulator_control::ManipulatorCommand::Request &req,
      manipulator_control::ManipulatorCommand::Response &res);
  bool callback_status_service(
      manipulator_control::ManipulatorCommand::Request &req,
      manipulator_control::ManipulatorCommand::Response &res);
};
#endif // MANIPULATOR_SERVICE_H

#ifndef MANIPULATOR_SERVICE_H
#define MANIPULATOR_SERVICE_H
#include "ManipulatorControl.h"
#include "manipulator_control/ManipulatorCommand.h"

#include "ros/node_handle.h"
class ManipulatorServiceHandler {
private:
  ManipulatorControlHandler<> &manipulator_control;
  ros::ServiceServer mode_service_;
  ros::ServiceServer nozzle_service_;
  ros::ServiceServer status_service_;

  // // Внутреннее состояние
  // uint8_t mode_{0};        // MANUAL
  // uint8_t nozzle_{0};      // NONE
  // uint8_t lock_status_{0}; // LOCKED
public:
  ManipulatorServiceHandler(ros::NodeHandle *node,
                            ManipulatorControlHandler<> &);
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

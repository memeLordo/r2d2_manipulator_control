#ifndef MANIPULATOR_SERVICE_H
#define MANIPULATOR_SERVICE_H
#include "ManipulatorControl.h"
#include "manipulator_control/ManipulatorCommand.h"

#include "ros/node_handle.h"
class ManipulatorServiceHandler {
private:
  ManipulatorControlHandler<> &m_manipulatorControl;

  ros::ServiceServer m_manipulatorService;

public:
  ManipulatorServiceHandler(ros::NodeHandle *node,
                            ManipulatorControlHandler<> &);

private:
  bool callbackService(manipulator_control::ManipulatorCommand::Request &req,
                       manipulator_control::ManipulatorCommand::Response &res);
  bool
  callbackModeService(manipulator_control::ManipulatorCommand::Request &req,
                      manipulator_control::ManipulatorCommand::Response &res);
  bool
  callbackNozzleService(manipulator_control::ManipulatorCommand::Request &req,
                        manipulator_control::ManipulatorCommand::Response &res);
  bool
  callbackStatusService(manipulator_control::ManipulatorCommand::Request &req,
                        manipulator_control::ManipulatorCommand::Response &res);
};
#endif // MANIPULATOR_SERVICE_H

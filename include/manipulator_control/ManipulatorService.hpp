#ifndef R2D2_MANIPULATOR_SERVICE_H
#define R2D2_MANIPULATOR_SERVICE_H

#include "ManipulatorControl.hpp"
#include "r2d2_msg_pkg/ManipulatorCommand.h"

class ManipulatorServiceHandler {
private:
  static constexpr const char *s_name = "ManipulatorService";

  ManipulatorControlHandler<> *m_manipulatorControl;
  const std::string m_serviceTopic;

  ros::ServiceServer m_manipulatorService;

public:
  ManipulatorServiceHandler(ros::NodeHandle *node,
                            ManipulatorControlHandler<> *);

private:
  bool callbackService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                       r2d2_msg_pkg::ManipulatorCommand::Response &res);
  bool callbackModeService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                           r2d2_msg_pkg::ManipulatorCommand::Response &res);
  bool callbackNozzleService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                             r2d2_msg_pkg::ManipulatorCommand::Response &res);
  bool callbackStatusService(r2d2_msg_pkg::ManipulatorCommand::Request &req,
                             r2d2_msg_pkg::ManipulatorCommand::Response &res);
};
#endif // R2D2_MANIPULATOR_SERVICE_H

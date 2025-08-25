// #include "manipulator_control/SetMode.h"
// #include "manipulator_control/SetNozzle.h"
// #include "manipulator_control/SetStatus.h"
#include <ros/ros.h>

class ManipulatorServiceHandler {
private:
  // Серверы сервисов
  ros::ServiceServer mode_service_;
  ros::ServiceServer nozzle_service_;
  ros::ServiceServer status_service_;

  // // Внутреннее состояние
  // uint8_t mode_{0};        // MANUAL
  // uint8_t nozzle_{0};      // NONE
  // uint8_t lock_status_{0}; // LOCKED

public:
  ManipulatorServiceHandler(ros::NodeHandle *nh);
  bool callback_mode_service(manipulator_control::SetMode::Request &req,
                             manipulator_control::SetMode::Response &res);
  bool callback_nozzle_service(manipulator_control::SetNozzle::Request &req,
                               manipulator_control::SetNozzle::Response &res);
  bool callback_status_service(manipulator_control::SetStatus::Request &req,
                               manipulator_control::SetStatus::Response &res);
};

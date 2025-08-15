#include "ShoulderHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include <ros/ros.h>

#define SHOULDER_INPUT_NODE "/manipulator/shoulder_input"
#define SHOULDER_OUTPUT_NODE "/manipulator/shoulder_output"

const double ShoulderHandler::coeffs[]{-0.00011, 0.341, -110.2};

ShoulderHandler::ShoulderHandler(ros::NodeHandle *node) : pipe(node) {
  subscriber = node->subscribe(SHOULDER_OUTPUT_NODE, 1000,
                               &ShoulderHandler::callback_shoulder, this);
  publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(SHOULDER_INPUT_NODE, 10);
  //
  // TODO: Добавить обновление данных радиуса при инициализации
  //
}

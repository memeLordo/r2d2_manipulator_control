#include "ShoulderHandler.h"
#include "Polynome.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include <ros/ros.h>

#define SHOULDER_INPUT_NODE "/manipulator/shoulder_input"
#define SHOULDER_OUTPUT_NODE "/manipulator/shoulder_output"

const double ShoulderHandler::coeffs[]{-0.00011, 0.341, -110.2};

ShoulderHandler::ShoulderHandler(ros::NodeHandle *node) : pipe(node) {
  subscriber = node->subscribe(SHOULDER_INPUT_NODE, 1000,
                               &ShoulderHandler::callback_shoulder, this);
  publisher =
      node->advertise<r2d2_msg_pkg::DriverState>(SHOULDER_OUTPUT_NODE, 10);
  //
  // TODO: Добавить обновление данных радиуса при инициализации
  //
}
void ShoulderHandler::callback_shoulder(
    const r2d2_msg_pkg::DriverCommand::ConstPtr &msg) {
  callback_params = shoulder_t{msg->omega, msg->theta};
}
double ShoulderHandler::get_angle() {
  //
  // TODO: Добавить проверку на данные радиуса
  // Если (pipe.radius < H_min) -> обновить данные нв ноде PipeHandler
  //
  return Horner::polynome(coeffs, pipe.get_radius());
}

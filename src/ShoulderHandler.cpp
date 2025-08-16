#include "ShoulderHandler.h"
#include "Polynome.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include <ros/ros.h>

#define SHOULDER_INPUT_NODE "/shoulder_input"
#define SHOULDER_OUTPUT_NODE "/shoulder_output"

const double ShoulderHandler::coeffs[]{-0.00011, 0.341, -110.2};
const double ShoulderHandler::length{10};

ShoulderHandler::ShoulderHandler(ros::NodeHandle *node) : pipe(node) {
  subscriber = node->subscribe(SHOULDER_OUTPUT_NODE, 1000,
                               &ShoulderHandler::callback_shoulder, this);
  publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(SHOULDER_INPUT_NODE, 10);
  //
  // TODO: Добавить обновление данных радиуса при инициализации
  //
}
template <typename T> T ShoulderHandler::calc_angle() {
  return static_cast<T>(Horner::polynome(coeffs, pipe.get_radius()));
}
template <typename T> T ShoulderHandler::calc_angle(T theta) {
  return static_cast<T>(Horner::polynome(coeffs, theta));
}

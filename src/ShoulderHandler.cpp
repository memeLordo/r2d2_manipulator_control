#include "ShoulderHandler.h"
#include "Polynome.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include <ros/ros.h>
#include <unistd.h>

#define SHOULDER_INPUT_NODE "/shoulder_input"
#define SHOULDER_OUTPUT_NODE "/shoulder_output"

template <typename T>
const T ShoulderHandler<T>::coeffs[]{0.00024, 0.142, 20.9};
template <typename T> const T ShoulderHandler<T>::length{5};

template <typename T>
ShoulderHandler<T>::ShoulderHandler(ros::NodeHandle *node,
                                    PipeHandler<T> &pipePtr)
    : pipe(pipePtr) {
  subscriber = node->subscribe(SHOULDER_OUTPUT_NODE, 1000,
                               &ShoulderHandler::callback_shoulder, this);
  publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(SHOULDER_INPUT_NODE, 10);
  //
  // TODO: Добавить обновление данных радиуса при инициализации
  //
}
template <typename T> T ShoulderHandler<T>::calc_angle() {
  return static_cast<T>(Horner::polynome(coeffs, pipe.get_radius()));
}
template <typename T> T ShoulderHandler<T>::calc_angle(T theta) {
  return static_cast<T>(Horner::polynome(coeffs, theta));
}

template class ShoulderHandler<>;

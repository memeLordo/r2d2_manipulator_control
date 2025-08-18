#include "ElbowHandler.h"
#include "Polynome.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include <ros/node_handle.h>

#define ELBOW_INPUT_NODE "/elbow_input"
#define ELBOW_OUTPUT_NODE "/elbow_output"

template <typename T> const T ElbowHandler<T>::coeffs[]{0.00024, 0.142, 20.9};
template <typename T> const T ElbowHandler<T>::length{5};

template <typename T>
ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node) : pipe(node) {
  subscriber = node->subscribe(ELBOW_OUTPUT_NODE, 1000,
                               &ElbowHandler::callback_elbow, this);
  publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(ELBOW_INPUT_NODE, 10);
}
template <typename T> T ElbowHandler<T>::calc_angle() {
  return static_cast<T>(Horner::polynome(coeffs, pipe.get_radius()));
}
template <typename T> T ElbowHandler<T>::calc_angle(T theta) {
  return static_cast<T>(Horner::polynome(coeffs, theta));
}

template class ElbowHandler<>;

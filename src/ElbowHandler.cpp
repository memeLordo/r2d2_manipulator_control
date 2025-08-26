#include "ElbowHandler.h"
#include "Polynome.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include <ros/node_handle.h>

constexpr const char *ELBOW_INPUT_NODE = "/elbow_input";
constexpr const char *ELBOW_OUTPUT_NODE = "/elbow_output";

template <typename T> const T ElbowHandler<T>::coeffs[]{0.00024, 0.142, 20.9};
template <typename T> const T ElbowHandler<T>::length{5};
// template<typename T> const T ElbowHandler<T>::angle_treshold{5};
template <typename T>
ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node,
                              const PipeHandler<T> &pipeRef)
    : pipe(pipeRef) {
  constexpr int QUEUE_SIZE = 8;
  subscriber = node->subscribe(ELBOW_OUTPUT_NODE, QUEUE_SIZE,
                               &ElbowHandler::callback_elbow, this);
  publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(ELBOW_INPUT_NODE,
                                                           QUEUE_SIZE);
}
template <typename T> T ElbowHandler<T>::calc_angle() {
  return static_cast<T>(
      Horner::polynome(coeffs, pipe.get_radius()) /* - angle_treshold*/);
}
template <typename T> T ElbowHandler<T>::calc_angle(T theta) {
  return static_cast<T>(Horner::polynome(coeffs, theta));
}

template class ElbowHandler<>;

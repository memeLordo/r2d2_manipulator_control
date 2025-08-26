#include "ElbowHandler.h"
#include "Polynome.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include <ros/node_handle.h>

constexpr const char *ELBOW_INPUT_NODE = "/elbow_input";
constexpr const char *ELBOW_OUTPUT_NODE = "/elbow_output";

template <typename T> const T ElbowHandler<T>::m_coeffs[]{0.00024, 0.142, 20.9};
template <typename T> const T ElbowHandler<T>::m_length{5};
// template<typename T> const T ElbowHandler<T>::angle_treshold{5};
template <typename T>
ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node,
                              const PipeHandler<T> &pipeRef)
    : m_pipe(pipeRef) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(ELBOW_OUTPUT_NODE, QUEUE_SIZE,
                                 &ElbowHandler::callbackElbow, this);
  m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(ELBOW_INPUT_NODE,
                                                             QUEUE_SIZE);
}
template <typename T> T ElbowHandler<T>::calcAngle() {
  return static_cast<T>(
      Horner::polynome(m_coeffs, m_pipe.getRadius()) /* - angle_treshold*/);
}
template <typename T> T ElbowHandler<T>::calcAngle(T theta) {
  return static_cast<T>(Horner::polynome(m_coeffs, theta));
}

template class ElbowHandler<>;

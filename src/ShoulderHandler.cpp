#include "ShoulderHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "utils/Math.h"
#include "utils/Polynome.h"
#include <ros/node_handle.h>

constexpr const char *SHOULDER_INPUT_NODE = "/shoulder_input";
constexpr const char *SHOULDER_OUTPUT_NODE = "/shoulder_output";

template <typename T>
const T ShoulderHandler<T>::m_coeffs[]{0.00024, 0.142, 20.9};
template <typename T> const T ShoulderHandler<T>::m_length{363}; // mm

template <typename T>
ShoulderHandler<T>::ShoulderHandler(ros::NodeHandle *node,
                                    const PipeHandler<T> &pipeRef)
    : m_pipe(pipeRef) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(SHOULDER_OUTPUT_NODE, QUEUE_SIZE,
                                 &ShoulderHandler::callbackShoulder, this);
  m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(
      SHOULDER_INPUT_NODE, QUEUE_SIZE);
}
template <typename T> T ShoulderHandler<T>::calcAngle() {
  T res = horner::polynome(m_coeffs, m_pipe.getRadius());
  ROS_DEBUG_STREAM("Shoulder::calcAngle() : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}
template <typename T> T ShoulderHandler<T>::calcAngle(T theta) {
  T res = horner::polynome(m_coeffs, theta);
  ROS_DEBUG_STREAM("Shoulder::calcAngle(" << WHITE(theta) << GREEN(") : ")
                                          << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template class ShoulderHandler<>;

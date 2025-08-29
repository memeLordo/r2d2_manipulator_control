#include "ShoulderHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "utils/Math.h"
#include "utils/Polynome.h"
#include <ros/node_handle.h>

template <typename T>
const std::string ShoulderHandler<T>::OUTPUT_NODE = "/shoulder_output";
template <typename T>
const std::string ShoulderHandler<T>::INPUT_NODE = "/shoulder_input";

template <typename T>
const T ShoulderHandler<T>::m_coeffs[]{0.00024, -0.142, 20.9};
template <typename T> const T ShoulderHandler<T>::m_length{363}; // mm
template <typename T> const T ShoulderHandler<T>::m_speed{100};

template <typename T>
ShoulderHandler<T>::ShoulderHandler(ros::NodeHandle *node) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(OUTPUT_NODE, QUEUE_SIZE,
                                 &ShoulderHandler::callbackShoulder, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(INPUT_NODE, QUEUE_SIZE);
}
template <typename T> T ShoulderHandler<T>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM("Shoulder::calcAngle(radius = " << WHITE(radius)
                                                   << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template class ShoulderHandler<>;

#include "ElbowHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "utils/Math.h"
#include "utils/Polynome.h"
#include <ros/node_handle.h>

template <typename T>
const std::string ElbowHandler<T>::INPUT_NODE = "/elbow_input";
template <typename T>
const std::string ElbowHandler<T>::OUTPUT_NODE = "/elbow_output";

template <typename T>
const T ElbowHandler<T>::m_coeffs[]{-0.00011, 0.341, -105.2};
template <typename T> const T ElbowHandler<T>::m_length{180}; // mm
template <typename T> const T ElbowHandler<T>::m_speed{100};
// template<typename T> const T ElbowHandler<T>::angle_treshold{5};

template <typename T> ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node) {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(OUTPUT_NODE, QUEUE_SIZE,
                                 &ElbowHandler::callbackElbow, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(INPUT_NODE, QUEUE_SIZE);
}
template <typename T> T ElbowHandler<T>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM("Elbow::calcAngle(radius = " << WHITE(radius)
                                                << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template class ElbowHandler<>;

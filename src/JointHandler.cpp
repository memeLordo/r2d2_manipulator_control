#include "JointHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "utils/Math.h"
#include "utils/Polynome.h"
#include <ros/node_handle.h>

// template <typename T>
// const std::string JointHandler<T>::INPUT_NODE = "/elbow_input";
// template <typename T>
// const std::string JointHandler<T>::OUTPUT_NODE = "/elbow_output";

// template <typename T>
// const T JointHandler<T>::m_coeffs[]{-0.00011, 0.341, -105.2};
// template <typename T> const T JointHandler<T>::m_length{180}; // mm
// template <typename T> const T JointHandler<T>::m_speed{100};
// template<typename T> const T ElbowHandler<T>::angle_treshold{5};

template <typename T>
JointHandler<T>::JointHandler(ros::NodeHandle *node)
    : m_length{}, m_speed{}, m_coeffs[]{0, 0, 0} {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(OUTPUT_NODE, QUEUE_SIZE,
                                 &JointHandler::callbackElbow, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(INPUT_NODE, QUEUE_SIZE);
}
template <typename T> T JointHandler<T>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM("Elbow::calcAngle(radius = " << WHITE(radius)
                                                << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template class JointHandler<>;

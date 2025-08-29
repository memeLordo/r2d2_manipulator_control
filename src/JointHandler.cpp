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
template <typename T, size_t N>
JointHandler<T, N>::JointHandler(ros::NodeHandle *node,
                                 const std::string &inputNode,
                                 const std::string &outputNode,
                                 std::initializer_list<T> coeffs,
                                 const T &length, const T &speed)
    : m_inputNode{inputNode}, m_outputNode{outputNode}, m_length{length},
      m_speed{speed}, m_coeffs{} {
  setCoeffsFrom(coeffs);
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(outputNode, QUEUE_SIZE,
                                 &JointHandler::callbackJoint, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(inputNode, QUEUE_SIZE);
}
template <typename T> T JointHandler<T>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM("Elbow::calcAngle(radius = " << WHITE(radius)
                                                << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template class JointHandler<>;

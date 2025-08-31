#include "JointHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "utils/Math.h"
#include "utils/Polynome.h"
#include <ros/node_handle.h>

template <typename T, size_t N>
JointHandler<T, N>::JointHandler(ros::NodeHandle *node,
                                 const std::string &inputNode,
                                 const std::string &outputNode, const T &length,
                                 const T &speed, const std::array<T, N> &coeffs)
    : m_inputNode{inputNode}, m_outputNode{outputNode}, m_length{length},
      m_speed{speed}, m_coeffs{1, 2, 3} {
  // setCoeffsFrom(coeffs);
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(outputNode, QUEUE_SIZE,
                                 &JointHandler::callbackJoint, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(inputNode, QUEUE_SIZE);
}
template <typename T, size_t N> T JointHandler<T, N>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM("Elbow::calcAngle(radius = " << WHITE(radius)
                                                << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}
template class JointHandler<>;

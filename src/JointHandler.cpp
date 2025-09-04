#include "JointHandler.h"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "utils/Math.h"
#include "utils/Polynome.h"
#include <ros/node_handle.h>

template <typename T>
JointHandler<T>::JointHandler(ros::NodeHandle *node,
                              const std::string &inputNode,
                              const std::string &outputNode, const T &length,
                              const T &speed, const std::vector<T> &coeffs)
    : m_inputNode{inputNode}, m_outputNode{outputNode}, m_length{length},
      m_speed{speed}, m_coeffs{coeffs} {
  constexpr int QUEUE_SIZE = 8;
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &JointHandler::callbackJoint, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputNode, QUEUE_SIZE);
}
template <typename T> T JointHandler<T>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM("Joint::calcAngle(radius = " << WHITE(radius)
                                                << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template class JointHandler<>;

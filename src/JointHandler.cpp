#include "JointHandler.hpp"
#include "r2d2_msg_pkg/DriverCommand.h"
#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.hpp"
#include "utils/Math.hpp"
#include "utils/Polynome.hpp"
#include <ros/node_handle.h>
#include <ros/topic.h>

template <typename T>
JointHandler<T>::JointHandler(ros::NodeHandle *node, const std::string &name,
                              const std::string &inputNode,
                              const std::string &outputNode, const T &length,
                              const T &speed, const std::vector<T> &coeffs)
    : m_name{name}, m_inputNode{inputNode}, m_outputNode{outputNode},
      m_length{length}, m_speed{speed}, m_coeffs{coeffs} {
  constexpr int QUEUE_SIZE = 8;
  ROS_INFO_STREAM(CYAN("Waiting for " << m_name << " topic..."));
  ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputNode);
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &JointHandler::callbackJoint, this);
  m_publisher =
      node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputNode, QUEUE_SIZE);
}
template <typename T> T JointHandler<T>::calcAngle(T radius) {
  T res = horner::polynome(m_coeffs, radius);
  ROS_DEBUG_STREAM(m_name << "::calcAngle(radius = " << WHITE(radius)
                          << ") : " << WHITE(res));
  return r2d2_math::max<T>(res, 0);
}

template <typename T>
ShoulderHandler<T>::ShoulderHandler(ros::NodeHandle *node)
    : JointHandler<T>(node, "Shoulder", "/shoulder_input", "/shoulder_output",
                      config::shoulder::length, config::shoulder::speed,
                      config::shoulder::coeffs) {}
template <typename T>
ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node)
    : JointHandler<T>(node, "Elbow", "/elbow_input", "/elbow_output",
                      config::elbow::length, config::elbow::speed,
                      config::elbow::coeffs) {}

template class JointHandler<>;
template class ShoulderHandler<>;
template class ElbowHandler<>;

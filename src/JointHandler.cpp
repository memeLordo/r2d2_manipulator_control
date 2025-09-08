#include "JointHandler.hpp"
#include "utils/Math.hpp"
#include "utils/Polynome.hpp"

template <typename T>
JointHandler<T>::JointHandler(ros::NodeHandle *node, const std::string &name,
                              const std::string &input,
                              const std::string &output)
    : IConfigJson<T>(name), m_name{name}, m_inputTopic{input},
      m_outputTopic{output}, m_length{getParam("length")},
      m_speed{getParam("speed")}, m_angleOffset{getParam("angle_offset")},
      m_angleTolerance{getParam("angle_tolerance")},
      m_coeffs{getVector("coeffs")} {
  waitForTopic();
  m_subscriber =
      node->subscribe(m_outputTopic, 10, &JointHandler::callbackJoint, this);
  m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputTopic, 10);
}
template <typename T> T JointHandler<T>::getTargetAngle(T radius) {
  const T theta_ = horner::polynome(m_coeffs, radius) + m_angleOffset;
  ROS_DEBUG_STREAM(m_name << "::calcAngle(radius = " << WHITE(radius)
                          << ") : " << WHITE(theta_));
  return r2d2_math::max<T>(theta_, 0);
}

template <typename T>
ShoulderHandler<T>::ShoulderHandler(ros::NodeHandle *node)
    : JointHandler<T>(node, "Shoulder", "/shoulder_input", "/shoulder_output") {
}
template <typename T>
ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node)
    : JointHandler<T>(node, "Elbow", "/elbow_input", "/elbow_output") {}

template class JointHandler<>;
template class ShoulderHandler<>;
template class ElbowHandler<>;

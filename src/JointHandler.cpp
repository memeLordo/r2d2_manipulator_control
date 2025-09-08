#include "JointHandler.hpp"
#include "utils/Math.hpp"
#include "utils/Polynome.hpp"

template <typename T>
JointHandler<T>::JointHandler(ros::NodeHandle *node, const std::string &name)
    : JointConfig<T>(name) {
  waitForTopic();
  m_subscriber =
      node->subscribe(m_outputTopic, 10, &JointHandler::callbackJoint, this);
  m_publisher = node->advertise<r2d2_msg_pkg::DriverCommand>(m_inputTopic, 10);
}
template <typename T> T JointHandler<T>::getTargetAngle(T radius) const {
  const T theta_ = horner::polynome(m_coeffs, radius) + m_angleOffset;
  ROS_DEBUG_STREAM(m_name << "::calcAngle(radius = " << WHITE(radius)
                          << ") : " << WHITE(theta_));
  return r2d2_math::max<T>(theta_, 0);
}

template <typename T>
ShoulderHandler<T>::ShoulderHandler(ros::NodeHandle *node)
    : JointHandler<T>(node, "Shoulder") {}
template <typename T>
ElbowHandler<T>::ElbowHandler(ros::NodeHandle *node)
    : JointHandler<T>(node, "Elbow") {}

template class JointHandler<>;
template class ShoulderHandler<>;
template class ElbowHandler<>;

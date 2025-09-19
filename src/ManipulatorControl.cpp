#include "ManipulatorControl.hpp"

using namespace r2d2_state;
using namespace r2d2_type;

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : ManipulatorConfig<T>{},
      m_pipe{node},
      m_payload{node},
      m_shoulder{node},
      m_elbow{node} {
  const double RATE = node->param<T>("control_rate", 20);
  ROS_DEBUG_STREAM("Set RATE: " << RATE);
  m_timer = node->createTimer(
      ros::Duration(1 / RATE),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(
    const ros::TimerEvent &) {
  std::lock_guard<std::mutex> lock(m_mutex);
  ROS_DEBUG_STREAM("\ncallbackManipulator()");

  switch (m_workMode.type) {
    case WorkMode::AUTO:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::AUTO"));
      processControl(m_pipe.getRadius(), m_payload.getForce());
      return;

    case WorkMode::STOP:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
      processStop();
      return;

    case WorkMode::MANUAL:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::MANUAL"));
      this->resetMode();
      return;

    default:
      ROS_DEBUG_STREAM(YELLOW("Pending mode"));
      return;
  }
}
template <typename T>
void ManipulatorControlHandler<T>::checkSetup(const T force) {
  if (!m_needsSetup) {
    ROS_DEBUG_STREAM_ONCE(CYAN("Control setup finished!"));
    return;
  }
  ROS_DEBUG_STREAM(MAGENTA("\ncheckSetup()"));
  const bool needsAngleControl_{m_shoulder.needsAngleControl() ||
                                m_elbow.needsAngleControl()};
  const bool needsForceControl_{force < getTargetForce(1.2f)};
  m_needsSetup = needsAngleControl_ && needsForceControl_;
  ROS_DEBUG_STREAM(CYAN("needsAngleControl_ = " << needsAngleControl_));
  ROS_DEBUG_STREAM(CYAN("needsForceControl_ = " << needsForceControl_));
  ROS_DEBUG_STREAM(CYAN("m_needsSetup = " << m_needsSetup));
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::checkSetup()"));
}
template <typename T>
void ManipulatorControlHandler<T>::processStop() {
  ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
  processAngleControl(getRadius());
  publishResults();
}
template <typename T>
void ManipulatorControlHandler<T>::processControl(const T radius,
                                                  const T force) {
  switch (m_lockStatus.type) {
    case LockStatus::UNLOCKED:
      ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
      checkSetup(force);
      processAngleControl(radius);
      processForceControl(force);
      break;
    default:
      break;
  }
  publishResults();
}
template <typename T>
void ManipulatorControlHandler<T>::processAngleControl(const T radius) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessRadiusControl()"));
  m_shoulder.updateAngleByRadius(radius);
  m_elbow.updateAngleByRadius(radius);
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processRadiusControl()"));
}
template <typename T>
void ManipulatorControlHandler<T>::processForceControl(const T force) {
  if (m_needsSetup) return;
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  m_elbow.setAngleByRadius(getCurrentRadius() + getForceDiff(force));
  // m_elbow.incrementAngleBy(getForceDiff(force));
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processForceControl()"));
}

template class ManipulatorControlHandler<>;

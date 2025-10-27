#include "ManipulatorControl.hpp"

#include "r2d2_utils_pkg/Math.hpp"

using namespace r2d2_state;
using namespace r2d2_type;

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle* node)
    : ManipulatorConfig<T>{},
      m_pipe{node},
      m_payload{node},
      m_joints{node, "shoulder", "elbow"} {
  ROS_DEBUG_STREAM(MAGENTA("ManipulatorControlHandler()"));
  const auto RATE_{node->param<float>("control_rate", 20)};
  assert(RATE_ > 0);
  ROS_DEBUG_STREAM("Set RATE: " << RATE_);
  m_timer = node->createTimer(
      ros::Duration(1 / RATE_),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(const ros::TimerEvent&) {
  ROS_DEBUG_STREAM("\ncallbackManipulator()");

  switch (m_workMode.type) {
    case WorkMode::SETUP:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::SETUP"));
      processSetup(m_pipe.getRadius(), m_payload.getForce());
      break;

    case WorkMode::AUTO:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::AUTO"));
      processControl(m_payload.getForce());
      break;

    case WorkMode::STOP:
      ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
      processStop();
      break;

    default:
      ROS_DEBUG_STREAM(YELLOW("Pending mode"));
      return;
  }
  m_joints.publish();
}
template <typename T>
void ManipulatorControlHandler<T>::processSetup(const T radius, const T force) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessSetup()"));
  if (!m_needsSetup) {
    ROS_DEBUG_STREAM_ONCE(CYAN("Control setup finished!"));
    m_joints.resetControlFlag();
    this->setMode(WorkMode::AUTO);
    return;
  }
  checkSetup(force);
  m_joints.setAngleByRadius(radius);
}
template <typename T>
void ManipulatorControlHandler<T>::processControl(const T force) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessControl()"));
  const T curentRadius_{getCurrentRadius()};
  switch (m_lockStatus.type) {
    case LockStatus::UNLOCKED:
      ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
      m_joints.setCallbackAngle();

      m_shoulder.updateControlFlag(curentRadius_);
      m_shoulder.setAngleByRadius(curentRadius_);

      m_elbow.incrementAngleBy(getForceDiffSign(force),
                               0.01 * r2d2_math::abs(getForceDiff(force)));
      return;

    default:
      return;
  }
}

template class ManipulatorControlHandler<>;

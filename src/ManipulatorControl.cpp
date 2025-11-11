#include "ManipulatorControl.hpp"

using namespace r2d2_state;
using namespace r2d2_type;

/**
 * @brief Constructs a ManipulatorControlHandler and initializes all components.
 * @tparam T Numeric type for calculations (default: double)
 * @param node Pointer to the ROS node handle
 * @details Initializes pipe, payload, and joint handlers, and sets up the control timer
 *          based on the control_rate parameter from the ROS parameter server.
 */
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
/**
 * @brief Timer callback function that processes manipulator control based on work mode.
 * @tparam T Numeric type for calculations (default: double)
 * @param event The timer event (unused)
 * @details Handles different work modes: SETUP (initial positioning), AUTO (automatic control),
 *          and STOP (reset to zero). Publishes joint commands after processing.
 */
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(const ros::TimerEvent&) {
  ROS_DEBUG_STREAM(MAGENTA("\ncallbackManipulator()"));

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
/**
 * @brief Processes the setup phase of manipulator control.
 * @tparam T Numeric type for calculations (default: double)
 * @param radius The current pipe radius
 * @param force The current payload force
 * @details Sets joint angles based on radius and checks if setup is complete.
 *          If setup is finished, switches to AUTO mode and resets control flags.
 */
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
/**
 * @brief Processes automatic control mode.
 * @tparam T Numeric type for calculations (default: double)
 * @param force The current payload force
 * @details Synchronizes joint angles with current positions, updates control flags,
 *          adjusts shoulder angle based on current radius, and adjusts elbow angle
 *          based on force difference to maintain target force.
 */
template <typename T>
void ManipulatorControlHandler<T>::processControl(const T force) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessControl()"));
  m_joints.setCallbackAngle();
  const T curentRadius_{getCurrentRadius()};
  const T forceDiff_{getTargetForceDiff(force)};

  m_shoulder.updateControlFlag(curentRadius_);
  m_shoulder.setAngleByRadius(curentRadius_);

  updateControlFlag(forceDiff_);
  m_elbow.incrementAngleBy(-forceDiff_, 0.01);
}
/**
 * @brief Processes the stop phase, resetting joints to zero position.
 * @tparam T Numeric type for calculations (default: double)
 * @details If no joints need control, resets setup flag and work mode.
 *          Otherwise, synchronizes angles and resets all joints to zero.
 */
template <typename T>
void ManipulatorControlHandler<T>::processStop() {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessStop()"));
  if (!m_joints.needsControlAny()) {
    m_needsSetup = true;
    this->resetMode();
    return;
  }
  m_joints.setCallbackAngle();
  m_joints.updateControlFlag(getCurrentRadius());
  m_joints.resetAngle();
}

template class ManipulatorControlHandler<>;

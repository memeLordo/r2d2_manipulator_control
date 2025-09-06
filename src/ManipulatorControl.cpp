#include "ManipulatorControl.hpp"
#include "utils/Config.hpp"

using namespace config;
using namespace r2d2_math;
using namespace r2d2_state;
using namespace r2d2_type;

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : m_pipe(node), m_payload(node), m_elbow(node), m_shoulder(node) {
  const double RATE = node->param<T>("control_rate", 20);
  ROS_DEBUG_STREAM("Set RATE: " << RATE);
  m_timer = node->createTimer(
      ros::Duration(1 / RATE),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(
    const ros::TimerEvent &) {
  ROS_DEBUG_STREAM(MAGENTA("\ncallbackManipulator()"));
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опубликовать все переменные
   */
  switch (m_workMode) {
  case WorkMode::AUTO:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::AUTO"));
    processControl();
    return;

  case WorkMode::MANUAL:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::MANUAL"));
    resetMode();
    return;

  case WorkMode::STOP:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
    processStop(getRadius());
    return;

  default:
    ROS_DEBUG_STREAM(YELLOW("Pending mode"));
    return;
  }
}

template <typename T>
void ManipulatorControlHandler<T>::processStop(const T radius) {
  ROS_DEBUG_STREAM(YELLOW("WorkMode::STOP"));
  updateAngles();
  processAngleControl();
  publishResults();
}
template <typename T> void ManipulatorControlHandler<T>::processControl() {
  updateAngles();
  setTargetRadius(m_pipe.getRadius());
  calcCurrentRadius();
  switch (m_lockStatus) {
  case LockStatus::UNLOCKED:
    ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
    processAngleControl();
    processForceControl(m_payload.getForce() - getTargetForce());
    break;
  default:
    break;
  }
  publishResults();
}
template <typename T> void ManipulatorControlHandler<T>::processAngleControl() {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessRadiusControl()"));
  const bool isElbowReached_ = m_elbow.checkAngleDiff(m_targetRadius);
  const bool isShoulderReached_ =
      m_shoulder.checkAngleDiff(m_targetRadius, false);

  ROS_DEBUG_STREAM(BLUE("isElbowReached_ = " << isElbowReached_));
  ROS_DEBUG_STREAM(BLUE("isShoulderReached_ = " << isShoulderReached_));
  if (isElbowReached_)
    m_elbow.updateAngleByRadius(m_targetRadius);
  if (isShoulderReached_)
    m_shoulder.updateAngleByRadius(m_targetRadius);

  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processRadiusControl()"));
}
template <typename T>
void ManipulatorControlHandler<T>::processForceControl(const T forceDiff) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  const T forceDiff_ = sqr(forceDiff) - sqr(getForceTolerance());
  const bool isForceReached_ = forceDiff_ > 0;

  ROS_DEBUG_STREAM(BLUE("isForceReached_ = " << isForceReached_));
  if (isForceReached_)
    m_elbow.updateAngleBy(sign(forceDiff_) * 0.1);
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processForceControl()"));
}
template <typename T> void ManipulatorControlHandler<T>::updateNozzleType() {
  ROS_DEBUG_STREAM(MAGENTA("updateNozzleType()"));
  switch (m_nozzleType) {
  case NozzleType::BRUSH:
    m_params = manipulator16_t<T>{brush::target_force, brush::force_tolerance,
                                  brush::radius0};
    return;
  case NozzleType::EMA:
    m_params = manipulator16_t<T>{ema::target_force, ema::force_tolerance,
                                  ema::radius0};
    return;
  default:
    return;
  }
}

template class ManipulatorControlHandler<>;

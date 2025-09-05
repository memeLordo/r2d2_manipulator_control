#include "ManipulatorControl.hpp"
#include "utils/Config.hpp"

using namespace r2d2_state;

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : m_pipe(node), m_payload(node), m_elbow(node), m_shoulder(node) {
  const T RATE = node->param<T>("control_rate", 20);
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
    publishResults();
    return;

  case WorkMode::MANUAL:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::MANUAL"));
    resetMode();
    return;

  default:
    ROS_DEBUG_STREAM(YELLOW("Pending mode"));
    return;
  }
}

template <typename T> void ManipulatorControlHandler<T>::processControl() {
  setTargetRadius(m_pipe.getRadius());
  calcCurrentRadius();
  updateAngles();
  switch (m_lockStatus) {
  case LockStatus::UNLOCKED:
    ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
    if (!processRadiusControl(m_targetRadius - m_currentRadius))
      return;
    processAngleControl(m_elbow.calcAngle(m_targetRadius) - m_elbow.getAngle());
    processForceControl(getTargetForce() - m_payload.getForce());
    break;
  default:
    return;
  }
}
template <typename T>
bool ManipulatorControlHandler<T>::processRadiusControl(T radiusDiff,
                                                        T radiusTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessRadiusControl()"));
  const bool isRadiusReached_ = radiusDiff < radiusTreshold;
  ROS_DEBUG_STREAM(BLUE("isRadiusReached_ = " << isRadiusReached_));
  ROS_DEBUG_STREAM_COND(isRadiusReached_, CYAN("OK!"));
  if (!isRadiusReached_) {
    ROS_DEBUG_STREAM(CYAN("UPDATING ANGLES"));
    m_elbow.updateAngleByRadius(m_targetRadius);
    m_shoulder.updateAngleByRadius(m_targetRadius);
  }
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processRadiusControl()"));
  return isRadiusReached_;
}
template <typename T>
void ManipulatorControlHandler<T>::processAngleControl(const T angleDiff,
                                                       const T angleTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessAngleControl()"));
  const bool isAngleReached_ =
      r2d2_math::abs(angleDiff + angleTreshold) >= angleTreshold;
  ROS_DEBUG_STREAM(BLUE("isAngleReached_ = " << isAngleReached_));
  ROS_DEBUG_STREAM_COND(isAngleReached_, CYAN("OK!"));
  if (isAngleReached_)
    m_shoulder.updateAngleByRadius(m_targetRadius);
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processAngleControl()"));
}
template <typename T>
void ManipulatorControlHandler<T>::processForceControl(const T forceDiff,
                                                       const T forceTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  const bool isForceHigh_ = forceDiff < forceTreshold;
  const bool isForceLow_ = forceDiff > -forceTreshold;
  ROS_DEBUG_STREAM(BLUE("isForceHigh_ = " << isForceHigh_));
  ROS_DEBUG_STREAM(BLUE("isForceLow_ = " << isForceLow_));
  ROS_DEBUG_STREAM_COND(isForceHigh_ || isForceLow_, CYAN("OK!"));
  if (isForceHigh_)
    m_shoulder.updateAngleBy(-0.1);
  else if (isForceLow_)
    m_shoulder.updateAngleBy(0.1);
  ROS_DEBUG_STREAM(RED("\nend") << MAGENTA("::processForceControl()"));
}
template <typename T> void ManipulatorControlHandler<T>::updateNozzleType() {
  ROS_DEBUG_STREAM(MAGENTA("updateNozzleType()"));
  switch (m_nozzleType) {
  case NozzleType::BRUSH:
    m_params = r2d2_type::manipulator16_t<T>{config::brush::target_force,
                                             config::brush::radius0};
    return;
  case NozzleType::EMA:
    m_params = r2d2_type::manipulator16_t<T>{config::ema::target_force,
                                             config::ema::radius0};
    return;
  default:
    return;
  }
}

template class ManipulatorControlHandler<>;

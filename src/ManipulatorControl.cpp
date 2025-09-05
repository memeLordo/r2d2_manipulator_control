#include "ManipulatorControl.hpp"

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
  // Ранний выход при отключенном автоматическом режиме
  case WorkMode::AUTO:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::AUTO"));
    //  Проверка блокировки
    m_elbow.updateAngle();
    m_shoulder.updateAngle();
    processControl();
    publishResults();
    return;

  case WorkMode::MANUAL:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::MANUAL"));
    // m_shoulder.control_word = 1;
    // m_elbow.control_word = 1;
    resetMode();
    return;

  default:
    ROS_DEBUG_STREAM(YELLOW("Pending mode"));
    return;
  }
}

template <typename T> void ManipulatorControlHandler<T>::calcCurrentRadius() {
  ROS_DEBUG_STREAM(MAGENTA("calcRadius()"));
  m_currentRadius = m_shoulder.getRadius() + m_elbow.getRadius() + getRadius();
  ROS_DEBUG_STREAM(RED("calcCurrentRadius()")
                   << " : " << WHITE(m_currentRadius));
}

template <typename T> void ManipulatorControlHandler<T>::processControl() {
  setTargetRadius(m_pipe.getRadius());
  calcCurrentRadius();
  const T elbowCurrentAngle_{m_elbow.getAngle()};
  const T payloadCurrentForce_{m_payload.getForce()};

  if (!processRadiusControl(m_targetRadius - m_currentRadius))
    return;
  switch (m_lockStatus) {
  case LockStatus::UNLOCKED:
    ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
    // Основная логика управления
    processAngleControl(m_elbow.calcAngle(m_targetRadius) - elbowCurrentAngle_);
    processForceControl(getTargetForce() - payloadCurrentForce_);
    break;
  default:
    // m_elbow.updateSpeed(0);
    m_elbow.updateAngle();
    m_shoulder.updateAngle();
  }
}
template <typename T>
bool ManipulatorControlHandler<T>::processRadiusControl(T radiusDiff,
                                                        T radiusTreshold) {
  ROS_DEBUG_STREAM(YELLOW("\nprocessRadiusControl()"));
  // TODO: Если заблокированиы - манипуляторы в 0
  // Обновляем скорости

  ROS_DEBUG_STREAM(CYAN("Checking for reach..."));
  // bool state_ = m_shoulder.checkAngleDiff() && m_elbow.checkAngleDiff() ||
  //               m_payload.getForce() > 20000;
  const bool isRadiusReached_ = radiusDiff < radiusTreshold;
  if (!isRadiusReached_) {
    ROS_DEBUG_STREAM(CYAN("UPDATING ANGLES"));
    m_elbow.updateAngleByRadius(m_targetRadius);
    m_shoulder.updateAngleByRadius(m_targetRadius);
  } else {
    ROS_DEBUG_STREAM(CYAN("OK!"));
  }
  return isRadiusReached_;
}
template <typename T>
void ManipulatorControlHandler<T>::processAngleControl(const T angleDiff,
                                                       const T angleTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessAngleControl()"));
  const bool isAngleReached_ =
      r2d2_math::abs(angleDiff + angleTreshold) >= angleTreshold;
  if (isAngleReached_) {
    // Success angle control
    m_shoulder.updateAngleByRadius(m_targetRadius);
  } else {
    // No success
    m_shoulder.updateAngle();
  }
}
template <typename T>
void ManipulatorControlHandler<T>::processForceControl(const T forceDiff,
                                                       const T forceTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  const bool isForceHigh_ = forceDiff > forceTreshold;
  const bool isForceLow_ = forceDiff < -forceTreshold;
  if (isForceHigh_) {
    m_shoulder.updateAngle(m_shoulder.getAngle() -
                           0.1); // TODO: increaseAngle(speed)
  } else if (isForceLow_) {
    m_shoulder.updateAngle(m_shoulder.getAngle() +
                           0.1); // TODO: decreaseAngle(speed)
  } else {
    ROS_DEBUG_STREAM(CYAN("No change."));
    m_shoulder.updateAngle();
    return;
  }
}
template <typename T> void ManipulatorControlHandler<T>::publishResults() {
  ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
  m_elbow.publish();
  m_shoulder.publish();
}
template <typename T> void ManipulatorControlHandler<T>::updateNozzleType() {
  ROS_DEBUG_STREAM(MAGENTA("updateNozzleType()"));
  switch (m_nozzleType) {
  case NozzleType::BRUSH:
    m_params = r2d2_type::manipulator16_t<T>{10000, 347.0};
    return;
  case NozzleType::EMA:
    m_params = r2d2_type::manipulator16_t<T>{15000, 331.0};
    return;
  default:
    return;
  }
}

template class ManipulatorControlHandler<>;

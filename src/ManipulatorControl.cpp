#include "ManipulatorControl.h"
#include "utils/Debug.h"
#include "utils/Math.h"

using namespace r2d2_state;

template <typename T> bool ManipulatorControlHandler<T>::setup() {
  if (finishSetup) {
    return true;
  }
  ROS_DEBUG_STREAM(YELLOW("\nsetup()"));
  // TODO: Если заблокированиы - манипуляторы в 0
  // Обновляем скорости

  ROS_DEBUG_STREAM(CYAN("Checking for reach..."));
  bool state_ = !m_shoulder.checkAngleDiff() || !m_elbow.checkAngleDiff() ||
                m_payload.getForce() > 20000;
  if (!state_) {
    ROS_DEBUG_STREAM(RED("No reach!"));
    ROS_DEBUG(" ");
    ROS_DEBUG_STREAM(CYAN("UPDATING ANGLES"));
    m_elbow.updateAngle(m_elbow.calcAngle()); // TODO: updateAngleByRadius()
    m_shoulder.updateAngle(m_shoulder.calcAngle());
  } else {
    ROS_DEBUG_STREAM(CYAN("OK!"));
    updateSetup();
  }
  return state_;
  // m_elbow.updateSpeed();
  // m_shoulder.updateSpeed();
}

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : m_payload(node), m_pipe(node), m_elbow(node, m_pipe),
      m_shoulder(node, m_pipe) {

  const T RATE = node->param<T>("control_rate", 20);
  ROS_DEBUG_STREAM("Set RATE: " << RATE);
  m_timer = node->createTimer(
      ros::Duration(1 / RATE),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}

template <typename T> void ManipulatorControlHandler<T>::updateNozzleType() {
  ROS_DEBUG_STREAM(MAGENTA("updateNozzleType()"));
  switch (m_nozzleType) {
  case NozzleType::BRUSH:
    m_params = r2d2_types::manipulator16_t<T>{10000, 347.0};
    return;
  case NozzleType::EMA:
    m_params = r2d2_types::manipulator16_t<T>{15000, 331.0};
    return;
  default:
    return;
  }
};
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(
    const ros::TimerEvent &) {
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опубликовать все переменные
   */
  ROS_DEBUG_STREAM(MAGENTA("\ncallbackManipulator()"));
  switch (m_workMode) {
  // Ранний выход при отключенном автоматическом режиме
  case WorkMode::AUTO:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::AUTO"));
    switch (m_lockStatus) {
    //  Проверка блокировки
    case LockStatus::UNLOCKED:
      ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
      // Основная логика управления
      // m_elbow.updateSpeed();
      // m_elbow.updateAngle();
      // m_shoulder.updateSpeed();
      // m_shoulder.updateAngle();
      processControl();
      publishResults();
      return;
    default:
      // m_elbow.updateSpeed(0);
      // m_elbow.publish();
      m_elbow.updateAngle();
      m_shoulder.updateAngle();

      return;
    }
    break;

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
template <typename T> T ManipulatorControlHandler<T>::calcCurrentRadius() {
  ROS_DEBUG_STREAM(MAGENTA("calcRadius()"));
  T radius{m_shoulder.getLength() * r2d2_math::sin(m_shoulder.getAngle()) +
           m_elbow.getLength() * r2d2_math::sin(m_elbow.getAngle()) +
           getRadius()};
  ROS_DEBUG_STREAM("calcRadius() : " << WHITE(radius));
  ROS_DEBUG(" ");
  return radius;
}
template <typename T> void ManipulatorControlHandler<T>::processControl() {
  if (!setup())
    return;
  processAngleControl(m_elbow.getAngle(), m_elbow.calcAngle());
  processForceControl(m_payload.getForce(), getForce());
}
template <typename T>
void ManipulatorControlHandler<T>::processAngleControl(const T currentAngle,
                                                       const T targetAngle,
                                                       const T angleTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessAngleControl()"));
  ROS_DEBUG_STREAM(CYAN("Calculating angles"));

  ROS_DEBUG_STREAM("ANGLE_THRESHOLD : " << angleTreshold);
  const auto angleDiff_ =
      r2d2_math::abs(currentAngle - (targetAngle + angleTreshold));
  ROS_DEBUG_STREAM("angle_diff : " << WHITE(angleDiff_));
  ROS_DEBUG_STREAM(RED("if(angel_dif >= ANGLE_THRESHOLD)"));
  if (angleDiff_ >= angleTreshold) {
    ROS_DEBUG_STREAM(CYAN("Success angle control"));
    m_shoulder.updateAngle(m_shoulder.calcAngle(calcCurrentRadius()));
  } else {
    ROS_DEBUG_STREAM(CYAN("No success."));
    m_shoulder.updateAngle();
  }
  // m_elbow.control_word = 10;
  // m_shoulder.control_word = 10;
}
template <typename T>
void ManipulatorControlHandler<T>::processForceControl(const T currentForce,
                                                       const T targetForce,
                                                       const T forceTreshold) {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  // const auto current_force = m_payload.getForce();
  // const auto target_force = getForce();
  const auto forceDiff_ = currentForce - targetForce;
  ROS_DEBUG_STREAM("current_force check");
  // TODO:
  // TODO: update speed setter
  if (forceDiff_ < -forceTreshold) {
    ROS_DEBUG_STREAM(CYAN("current_force < target_force!"));
    m_shoulder.updateAngle(m_shoulder.calcAngle(calcRadius()) + 0.1);
  } else if (forceDiff_ > forceTreshold) {
    ROS_DEBUG_STREAM(CYAN("current_force > target_force!"));
    m_shoulder.updateAngle(m_shoulder.calcAngle(calcRadius()) - 0.1);
  } else {
    ROS_DEBUG_STREAM(CYAN("No change."));
    m_shoulder.updateAngle();
    // TODO: control_word = 1
    // For elbow and shoulder
  }
  // m_elbow.control_word = 10;
  // m_shoulder.control_word = 10;
}
template <typename T> void ManipulatorControlHandler<T>::publishResults() {
  ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
  m_elbow.publish();
  m_shoulder.publish();
}

template class ManipulatorControlHandler<>;

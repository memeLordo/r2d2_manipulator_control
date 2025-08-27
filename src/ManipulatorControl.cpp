#include "ManipulatorControl.h"
#include "utils/Debug.h"
#include "utils/Math.h"

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : m_payload(node), m_pipe(node), m_elbow(node, m_pipe),
      m_shoulder(node, m_pipe) {

  const T RATE = node->param<T>("control_rate", 20);
  ROS_DEBUG_STREAM("Set RATE: " << RATE);

  setMode(WorkMode::MANUAL);
  m_timer = node->createTimer(
      ros::Duration(2 / RATE),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(
    const ros::TimerEvent &) {
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
      processAngleControl();
      processForceControl();
      publishResults();
      return;
    default:
      m_elbow.updateSpeed(0);
      m_elbow.publish();
      return;
    }
    break;

  case WorkMode::MANUAL:
    ROS_DEBUG_STREAM(YELLOW("WorkMode::MANUAL"));
    setup();
    resetMode();
    return;

  default:
    return;
  }
}
template <typename T> void ManipulatorControlHandler<T>::setup() {
  ROS_DEBUG_STREAM(MAGENTA("\nsetup()"));
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опубликовать все переменные
   */
  updateJointState();
  publishJointState();

  // T current_angle[]{elbow.get_angle(), shoulder.get_angle()};
  // while ((elbow.get_state() - elbow.get_input_angle()) > 0.1 &&
  //        abs(shoulder.get_state() - shoulder.get_input_angle()) > 0.1) {
  //   ros::Duration(0.1).sleep();
  // }
  // TODO: while проверка DriverState == DriverCommand
  // exec wait... 0.1s
  // TODO: creaate async task for timer to wait
  // wait_for_state(elbow_state, shoulder_state);
  ROS_INFO("Manipulator Setup completed");
}

template <typename T> void ManipulatorControlHandler<T>::updateNozzleType() {
  ROS_DEBUG_STREAM(MAGENTA("updateNozzleType()"));
  switch (m_nozzleType) {
  case NozzleType::BRUSH:
    m_params = manipulator_t{100, 347.0};
    return;
  case NozzleType::EMA:
    m_params = manipulator_t{150, 331.0};
    return;
  default:
    return;
  }
};
template <typename T> T ManipulatorControlHandler<T>::calcRadius() {
  ROS_DEBUG_STREAM(MAGENTA("calcRadius()"));
  T radius = m_shoulder.getLength() * r2d2_math::sin(m_shoulder.getAngle()) +
             m_elbow.getLength() * r2d2_math::sin(m_elbow.getAngle()) +
             getRadius();
  ROS_DEBUG_STREAM("Radius : " << WHITE(radius));
  ROS_DEBUG(" ");
  return radius;
}
template <typename T> void ManipulatorControlHandler<T>::processAngleControl() {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessAngleControl()"));
  static constexpr T ANGLE_THRESHOLD = 5.0;

  const auto angle_diff =
      r2d2_math::abs(m_elbow.getAngle() - (m_elbow.calcAngle() -
                                           5.0 /*- m_elbow.getAngleMargin()*/));
  ROS_DEBUG_STREAM("angle_diff : " << WHITE(angle_diff));
  if (angle_diff >= ANGLE_THRESHOLD) { // TODO: fix to angle_threshold2
    ROS_DEBUG_STREAM(CYAN("Success angle control"));
    m_shoulder.updateAngle(m_shoulder.calcAngle(calcRadius()));
    m_shoulder.setPublishPending();
  }
}
template <typename T> void ManipulatorControlHandler<T>::processForceControl() {
  ROS_DEBUG_STREAM(MAGENTA("\nprocessForceControl()"));
  const auto current_force = m_payload.getForce();
  const auto target_force = getForce();
  ROS_DEBUG_STREAM_COND(current_force > target_force,
                        RED("Over force control"));
  ROS_DEBUG_STREAM_COND(current_force < target_force,
                        RED("Under force control"));
  // TODO: update speed setter
  if (current_force > target_force) {
    m_elbow.updateSpeed(-abs(m_elbow.getSpeed()));
  } else if (current_force < target_force) {
    m_elbow.updateSpeed();
  }
}
template <typename T> void ManipulatorControlHandler<T>::publishResults() {
  ROS_DEBUG_STREAM(MAGENTA("\npublishResults()"));
  m_elbow.publish();
  if (m_shoulder.isPublishPending()) {
    m_shoulder.publish();
    m_shoulder.clearPublishPending();
  }
}
template <typename T> void ManipulatorControlHandler<T>::updateJointState() {
  ROS_DEBUG_STREAM(MAGENTA("updateJointState()"));
  switch (m_lockStatus) {
  case LockStatus::UNLOCKED:
    ROS_DEBUG_STREAM(YELLOW("LockStatus::UNLOCKED"));
    // Вычисляем углы для разблокированного состояния
    m_elbow.updateAngle(m_elbow.calcAngle());
    m_shoulder.updateAngle(m_shoulder.calcAngle());
    break;
  default:
    ROS_DEBUG_STREAM(YELLOW("LockStatus::Default"));
    // Используем текущие значения для заблокированного состояния
    m_elbow.updateAngle();
    m_shoulder.updateAngle();
  }
  // TODO: Если заблокированиы - манипуляторы в 0
  // Обновляем скорости
  m_elbow.updateSpeed();
  m_shoulder.updateSpeed();
}
template <typename T> void ManipulatorControlHandler<T>::publishJointState() {
  ROS_DEBUG_STREAM(MAGENTA("publishJointState()"));
  m_elbow.publish();
  m_shoulder.publish();
}

template class ManipulatorControlHandler<>;

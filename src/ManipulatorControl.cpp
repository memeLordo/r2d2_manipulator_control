#include "ManipulatorControl.h"
#include <cmath>
#include <cstdlib>

constexpr double RATE = 20; // Hz

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : m_payload(node), m_pipe(node), m_elbow(node, m_pipe),
      m_shoulder(node, m_pipe) {

  const T RATE = node->param<T>("control_rate", 1);

  setMode(WorkMode::MANUAL);
  m_timer = node->createTimer(
      ros::Duration(1 / RATE),
      &ManipulatorControlHandler<T>::callbackManipulator, this);
}
template <typename T>
void ManipulatorControlHandler<T>::callbackManipulator(
    const ros::TimerEvent &) {
  switch (m_workMode) {
  // Ранний выход при отключенном автоматическом режиме
  case WorkMode::AUTO:
    switch (m_lockStatus) {
    //  Проверка блокировки
    case LockStatus::UNLOCKED:
      // Основная логика управления
      // ROS_INFO("Calculating angles");
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
    setup();
    resetMode();
    return;

  default:
    return;
  }
}
template <typename T> void ManipulatorControlHandler<T>::setup() {
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опубликовать все переменные
   */
  updateNozzleType();
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
  return m_shoulder.getLength() * sin(m_shoulder.getAngle()) +
         m_elbow.getLength() * sin(m_elbow.getAngle()) + getRadius();
}
template <typename T> void ManipulatorControlHandler<T>::processAngleControl() {
  static constexpr T ANGLE_THRESHOLD = 5.0;

  const T angle_diff =
      abs(m_elbow.getAngle() - m_elbow.calcAngle() - ANGLE_THRESHOLD);

  if (angle_diff >= ANGLE_THRESHOLD) { // TODO: fix to angle_threshold2
    m_shoulder.updateAngle(m_shoulder.calcAngle(calcRadius()));
    m_shoulder.setPublishPending();
  }
}
template <typename T> void ManipulatorControlHandler<T>::processForceControl() {
  const auto current_force = m_payload.getForce();
  const auto target_force = getForce();

  if (current_force > target_force) {
    m_elbow.updateSpeed(-abs(m_elbow.getSpeed()));
  } else if (current_force < target_force) {
    m_elbow.updateSpeed();
  } else {
    m_elbow.updateSpeed(0);
  }
}
template <typename T> void ManipulatorControlHandler<T>::publishResults() {
  m_elbow.publish();
  if (m_shoulder.isPublishPending()) {
    m_shoulder.publish();
    m_shoulder.clearPublishPending();
  }
}
template <typename T> void ManipulatorControlHandler<T>::updateJointState() {
  switch (m_lockStatus) {
  case LockStatus::UNLOCKED:
    // Вычисляем углы для разблокированного состояния
    m_elbow.updateAngle(m_elbow.calcAngle());
    m_shoulder.updateAngle(m_shoulder.calcAngle());
    break;
  default:
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
  m_elbow.publish();
  m_shoulder.publish();
}

template class ManipulatorControlHandler<>;

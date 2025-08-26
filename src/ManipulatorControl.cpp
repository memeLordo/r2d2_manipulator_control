#include "ManipulatorControl.h"
#include <cmath>
#include <cstdlib>

constexpr double RATE = 50; // Hz

template <typename T> T ManipulatorControlHandler<T>::calc_radius() {
  return shoulder.get_length() * sin(shoulder.get_angle()) +
         elbow.get_length() * sin(elbow.get_angle()) + get_radius();
}

template <typename T>
void ManipulatorControlHandler<T>::process_angle_control() {
  static constexpr T ANGLE_THRESHOLD = 5.0;

  const T angle_diff =
      abs(elbow.get_angle() - elbow.calc_angle() - ANGLE_THRESHOLD);

  if (angle_diff >= ANGLE_THRESHOLD) { // TODO: fix to angle_threshold2
    shoulder.update_angle(shoulder.calc_angle(calc_radius()));
    shoulder.set_publish_pending();
  }
}
template <typename T>
void ManipulatorControlHandler<T>::process_force_control() {
  const auto current_force = payload.get_force();
  const auto target_force = get_force();

  if (current_force > target_force) {
    elbow.update_speed(-abs(elbow.get_speed()));
  } else if (current_force < target_force) {
    elbow.update_speed();
  } else {
    elbow.update_speed(0);
  }
}
template <typename T> void ManipulatorControlHandler<T>::publish_results() {
  elbow.publish();
  if (shoulder.is_publish_pending()) {
    shoulder.publish();
    shoulder.clear_publish_pending();
  }
}

template <typename T> void ManipulatorControlHandler<T>::update_joint_state() {
  switch (status) {
  case LockStatus::UNLOCKED:
    // Вычисляем углы для разблокированного состояния
    elbow.update_angle(elbow.calc_angle());
    shoulder.update_angle(shoulder.calc_angle());
    break;
  default:
    // Используем текущие значения для заблокированного состояния
    elbow.update_angle();
    shoulder.update_angle();
  }
  // TODO: Если заблокированиы - манипуляторы в 0
  // Обновляем скорости
  elbow.update_speed();
  shoulder.update_speed();
}
template <typename T> void ManipulatorControlHandler<T>::publish_joint_state() {
  elbow.publish();
  shoulder.publish();
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
  update_joint_state();
  publish_joint_state();

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
template <typename T>
void ManipulatorControlHandler<T>::callback_manipulator(
    const ros::TimerEvent &) {
  switch (mode) {
  // Ранний выход при отключенном автоматическом режиме
  case WorkMode::AUTO:
    switch (status) {
    //  Проверка блокировки
    case LockStatus::UNLOCKED:
      // Основная логика управления
      // ROS_INFO("Calculating angles");
      process_angle_control();
      process_force_control();
      publish_results();
      return;
    default:
      elbow.update_speed(0);
      elbow.publish();
      return;
    }
    break;

  case WorkMode::MANUAL:
    setup();
    reset_mode();
    return;

  default:
    return;
  }
}

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : payload(node), pipe(node), elbow(node, pipe), shoulder(node, pipe) {
  timer = node->createTimer(ros::Duration(1 / RATE),
                            &ManipulatorControlHandler<T>::callback_manipulator,
                            this);
}

template class ManipulatorControlHandler<>;

#include "ManipulatorControl.h"
#include <cmath>
#include <cstdlib>

constexpr double TIME = 0.02; // s

template <typename T> bool ManipulatorControlHandler<T>::init_mode() {
  set_mode(AUTO); // TODO: получать из параметра или сервиса
  // TODO: добавить ожидание и проверку успешности установки
  return true; // Временно считаем всегда успешным
}
template <typename T> bool ManipulatorControlHandler<T>::init_nozzle() {
  set_nozzle(BRUSH); // TODO: получать из параметра или сервиса
  // TODO: добавить ожидание и проверку успешности установки
  return true; // Временно считаем всегда успешным
}
template <typename T> bool ManipulatorControlHandler<T>::init_lock() {
  set_lock(UNLOCKED); // TODO: добавить проверку с ros::ServiceServer()
  // TODO: добавить ожидание и проверку статуса блокировки
  return true; // Временно считаем всегда успешным
}

template <typename T> T ManipulatorControlHandler<T>::calc_radius() {
  return shoulder.get_length() * sin(shoulder.get_angle()) +
         elbow.get_length() * sin(elbow.get_angle()) + get_radius();
}

template <typename T>
void ManipulatorControlHandler<T>::process_angle_control() {
  static constexpr T ANGLE_THRESHOLD = 5.0;

  const T angle_diff =
      std::abs(elbow.get_angle() - elbow.calc_angle() - ANGLE_THRESHOLD);

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
    elbow.update_speed(-std::abs(elbow.get_speed()));
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

template <typename T> void ManipulatorControlHandler<T>::update_all() {
  switch (status) {
  case UNLOCKED:
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
template <typename T> void ManipulatorControlHandler<T>::publish_all() {
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
  if (!init_mode()) {
    ROS_ERROR("Failed to initialize manipulator mode");
    return;
  }
  if (!init_nozzle()) {
    ROS_WARN("Failed to set nozzle, using default");
  }
  if (!init_lock()) {
    ROS_WARN("Failed to set lock status, using default");
  }
  update_all();
  publish_all();
  // TODO: while проверка DriverState == DriverCommand
  // exec wait... 0.1s
  ROS_INFO("Manipulator setup completed");
}
template <typename T> bool ManipulatorControlHandler<T>::check_angle(T margin) {
  return std::abs(shoulder.get_angle() - shoulder.get_current_angle()) < margin;
}
template <typename T>
void ManipulatorControlHandler<T>::callback_manipulator(
    const ros::TimerEvent &) {
  switch (mode) {
  // Ранний выход при отключенном автоматическом режиме
  case AUTO:
    // TODO: добавить проверку достижения желаемого угла
    // check_angle()
    if (check_angle(T{1})) {
      return;
    }
    switch (status) {
    //  Проверка блокировки
    case UNLOCKED:
      // Основная логика управления
      process_angle_control();
      process_force_control();
      publish_results();
      return;
    case LOCKED:
      elbow.update_speed(0);
      elbow.publish();
      return;
    }
    break;

  case MANUAL:
    setup();
    return;
  }
  // Проверка блокировки
  // Ранний выход при отключенном автоматическом режиме
  // if (!mode) {
  //   setup();
  //   return;
  // }
  // Проверка блокировки
  // if (!status) {
  //   elbow.update_speed(0);
  //   elbow.publish();
  //   return;
  // }
  // // Основная логика управления
  // process_angle_control();
  // process_force_control();
  // publish_results();
}

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : payload(node), pipe(node), elbow(node, pipe), shoulder(node, pipe) {
  // setup();
  timer = node->createTimer(ros::Duration(TIME),
                            &ManipulatorControlHandler<T>::callback_manipulator,
                            this);
}

template class ManipulatorControlHandler<>;

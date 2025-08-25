#include "ManipulatorControl.h"
#include <cmath>
#include <cstdlib>
constexpr double RATE = 1; // Hz

template <typename T> bool ManipulatorControlHandler<T>::call_mode() {
  if (!mode_client_.exists()) {
    ROS_ERROR("set_mode service not available");
    return false;
  }

  srv_mode.request.mode = 0;
  ROS_INFO("call_mode::sent request");
  if (mode_client_.call(srv_mode)) {
    ROS_INFO("call_mode if call");
    if (srv_mode.response.success) {
      ROS_INFO("Mode service call successful: %s",
               srv_mode.response.message.c_str());
      // Обновление локального состояния
      set_mode(mode);
      return true;
    } else {
      ROS_ERROR("Mode service call failed: %s",
                srv_mode.response.message.c_str());
      return false;
    }
  } else {
    ROS_ERROR("Failed to call set_mode service");
    return false;
  }
}

template <typename T> bool ManipulatorControlHandler<T>::call_nozzle() {
  // if (!nozzle_client_.exists()) {
  //   ROS_ERROR("set_nozzle service not available");
  //   return false;
  // }
  //
  // srv_nozzle.request.nozzle_type = nozzle;
  //
  // if (nozzle_client_.call(srv_nozzle)) {
  //   if (srv_nozzle.response.success) {
  //     ROS_INFO("Nozzle service call successful: %s",
  //              srv_nozzle.response.message.c_str());
  //     // Обновление локального состояния
  //     set_nozzle(nozzle);
  //     return true;
  //   } else {
  //     ROS_ERROR("Nozzle service call failed: %s",
  //               srv_nozzle.response.message.c_str());
  //     return false;
  //   }
  // } else {
  //   ROS_ERROR("Failed to call set_nozzle service");
  //   return false;
  // }
  set_nozzle(BRUSH);
  return true;
}

template <typename T> bool ManipulatorControlHandler<T>::call_status() {
  // if (!status_client_.exists()) {
  //   ROS_ERROR("set_status service not available");
  //   return false;
  // }
  //
  // srv_status.request.lock_status = status;
  //
  // if (status_client_.call(srv_status)) {
  //   if (srv_status.response.success) {
  //     ROS_INFO("Status service call successful: %s",
  //              srv_status.response.message.c_str());
  //     // Обновление локального состояния
  //     set_lock(status);
  //     return true;
  //   } else {
  //     ROS_ERROR("Status service call failed: %s",
  //               srv_status.response.message.c_str());
  //     return false;
  //   }
  // } else {
  //   ROS_ERROR("Failed to call set_status service");
  //   return false;
  // }
  set_lock(LOCKED);
  return true;
}

// template <typename T> bool ManipulatorControlHandler<T>::init_mode() {
//   set_mode(AUTO); // TODO: получать из параметра или сервиса
//   // TODO: добавить ожидание и проверку успешности установки
//   return true; // Временно считаем всегда успешным
// }
// template <typename T> bool ManipulatorControlHandler<T>::init_nozzle() {
//   set_nozzle(BRUSH); // TODO: получать из параметра или сервиса
//   // TODO: добавить ожидание и проверку успешности установки
//   return true; // Временно считаем всегда успешным
// }
// template <typename T> bool ManipulatorControlHandler<T>::init_lock() {
//   set_lock(UNLOCKED); // TODO: добавить проверку с ros::ServiceServer()
//   // TODO: добавить ожидание и проверку статуса блокировки
//   return true; // Временно считаем всегда успешным
// }

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
  if (!call_mode()) {
    ROS_ERROR("Failed to initialize manipulator mode");
    return;
  }
  // if (!call_nozzle()) {
  //   ROS_WARN("Failed to set nozzle, using default");
  // }
  // if (!call_status()) {
  //   ROS_WARN("Failed to set lock status, using default");
  // }
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
  timer = node->createTimer(ros::Duration(1 / RATE),
                            &ManipulatorControlHandler<T>::callback_manipulator,
                            this);
  ROS_INFO("ManipulatorControlHandler::ManipulatorControlHandler");
}

template class ManipulatorControlHandler<>;

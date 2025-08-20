#include "ManipulatorControl.h"
#include <cmath>
#include <cstdlib>

#define TIME 0.02 // s

template <typename T> bool ManipulatorControlHandler<T>::init_mode() {
  set_mode(AUTO); // TODO: получать из параметра или сервиса
  // TODO: добавить ожидание и проверку успешности установки
  return true; // Временно считаем всегда успешным
}
template <typename T> bool ManipulatorControlHandler<T>::init_nozzle() {
  set_nozzle(BRUSH); // TODO: получать из параметра или сервиса
  update_all();      // Обновляем параметры после смены насадки
  // TODO: добавить ожидание и проверку успешности установки
  return true; // Временно считаем всегда успешным
}
template <typename T> bool ManipulatorControlHandler<T>::init_lock() {
  set_lock(UNLOCKED); // TODO: добавить проверку с ros::ServiceServer()
  // TODO: добавить ожидание и проверку статуса блокировки
  return true; // Временно считаем всегда успешным
}

template <typename T> auto ManipulatorControlHandler<T>::calc_radius() {
  return shoulder.get_length() * sin(shoulder.get_angle()) +
         elbow.get_length() * sin(elbow.get_angle()) + get_radius();
}

template <typename T>
ManipulatorControlHandler<T>::ManipulatorControlHandler(ros::NodeHandle *node)
    : pipe(node), payload(node), elbow(node, pipe), shoulder(node, pipe) {
  setup();
  timer = node->createTimer(ros::Duration(TIME),
                            &ManipulatorControlHandler<T>::callback_manipulator,
                            this);
}
template <typename T> void ManipulatorControlHandler<T>::setup() {
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опкбликовать все переменные
   */
SET_MODE:
  // TODO: добавить запрос, ожидание и проверку значений
  set_mode(AUTO);
  if (!mode)
    goto SET_MODE;

SET_NOZZLE:
  // TODO: добавить запросб ожидание и проверку значений
  set_nozzle(BRUSH);
  update_all();

SET_LOCK:
  // TODO: добавить запросб ожидание и проверку значений
  set_lock(UNLOCKED);

SET_VALUES:
  switch (status) {
  case UNLOCKED:
    elbow.update_angle(elbow.calc_angle());
    shoulder.update_angle(shoulder.calc_angle());
    break;
  case LOCKED:
    elbow.update_angle();
    shoulder.update_angle();
    break;
  }
  elbow.update_speed();
  shoulder.update_speed();

PUBLISH:
  publish_all();
}
template <typename T>
void ManipulatorControlHandler<T>::callback_manipulator(
    const ros::TimerEvent &) {
  /**
   * INFO:
   * 1. Проверка автоматического разжатия
   * 2. Проверка блокировки
   * 3. Проверка ограничения угла
   * 4. Обновить позицию плеча
   * 5. Проверка полученной нагрузки
   * 6. Опубликовать переменные
   */
SETUP_LOCALS:
  bool refresh{false};
  auto current_force = payload.get_force();

CHECK_MODE:
  if (!mode) {
    setup();
    return;
  }

CHECK_STATUS:
  if (!status) {
    elbow.update_speed(0);
    goto PUBLISH;
  }

SET_VALUES:
  if (abs(elbow.get_angle() - elbow.calc_angle() + 5.0) >= 5.0) {
    refresh = true;
    shoulder.update_angle(shoulder.calc_angle(calc_radius()));
  }
  if (current_force > get_force())
    elbow.update_speed(-elbow.get_speed());
  else if (current_force < get_force())
    elbow.update_speed();
  else
    elbow.update_speed(0);

PUBLISH:
  elbow.publish();
  if (refresh)
    shoulder.publish();
}

template class ManipulatorControlHandler<>;

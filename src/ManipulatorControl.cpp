#include "ManipulatorControlHandler.h"
#include <cmath>
#include <cstdlib>
#include <ros/ros.h>

void ManipulatorControlHandler::update() {
  switch (nozzle) {
  case BRUSH:
    params = manipulator_t{100, 347.0};
    break;
  case EMA:
    params = manipulator_t{150, 331.0};
    break;
  default:
    // TODO: throw error;
    break;
  }
}
void ManipulatorControlHandler::update_all() {
  update();
  elbow.update();
  shoulder.update();
}
void ManipulatorControlHandler::publish_all() {
  elbow.publish();
  shoulder.publish();
}
auto ManipulatorControlHandler::calc_radius() {
  return shoulder.get_length() * sin(shoulder.get_angle()) +
         elbow.get_length() * sin(elbow.get_angle()) + get_radius();
}

ManipulatorControlHandler::ManipulatorControlHandler(ros::NodeHandle *node)
    : payload(node), elbow(node), shoulder(node) {
  setup();
}
void ManipulatorControlHandler::setup() {

SETUP_MODE:
  /**
   * INFO:
   * 0. Получить данные для манипулятора (и трубы)
   * 1. Проверка автоматического разжатия
   * 2. Приём типа насадки (Щётка/ЕМА)
   * 3. Проверка статуса блокировки
   * 4. Обновить оставшиеся переменные
   * 5. Опкбликовать все переменные
   */
  // TODO: добавить запрос, ожидание и проверку значений
  set_mode(AUTO);

  if (!mode)
    goto SETUP_MODE;

  // TODO: добавить запросб ожидание и проверку значений
  set_nozzle(BRUSH);

  update_all();

  // TODO: добавить запросб ожидание и проверку значений
  set_lock(UNLOCKED);

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
  publish_all();
}
void ManipulatorControlHandler::callback_manipulator() {
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

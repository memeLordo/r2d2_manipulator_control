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
   * TODO:
   * 1. Проверка автоматического разжатия
   *  if (!manipulator.is_auto())
   *    setup();
   * TODO:
   * 2. Проверка блокировки
   *  if (manipulator.is_locked())
   *   elbow.update_omega(0);
   * TODO:
   * 3. Проверка ограничения угла:
   *  else{
   *   if (|elbow.get_input_angle() - elbow.calc_angle(pipe) -5| < 5)
   *   {
   * TODO:
   * 4. Обновить позицию плеча
   *    shoulder.set_refresh(True)
   *    shoulder.update_angle( shoulder.calc_angle(calc_radius(elbow, shoulder))
   * )
   *   }
   * TODO:
   * 5. Проверка полученной нагрузки
   *  input_force = get_input_force()
   *  if input_force > manipulator.force_needed
   *    elbow.update_omega(-elbow.get_input_omega())
   *  elif input_force < manipulator.force_needed
   *    elbow.update_omega()
   *  else
   *    elbow.update_omega(0)
   * TODO:
   * 6. Опубликовать переменные
   *
   * TODO:
   *  double calc_radius(ElbowHandler::elbow, ShoulderHandler::shoulder){
   *     return shoulder.get_length(??) * sin(shoulder.get_input_angle())
   * + elbow.get_length(??) * sin(elbow.get_input_angle());
   *      }
   */
}

#include "ManipulatorControlHandler.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>

ManipulatorControlHandler::ManipulatorControlHandler(ros::NodeHandle *node)
    : elbowHandler(node), shoulderHandler(node) {
  subscriber =
      node->subscribe("/manupulator/payload_input", 1000,
                      &ManipulatorControlHandler::callback_manipulator, this);
  publisher =
      node->advertise<std_msgs::Int64>("/manipulator/payload_output", 10);
  setup();
}
void ManipulatorControlHandler::setup() {
  /**
   * TODO:
   * 0. Получить данные для манипулятора (и трубы)
   *  update()
   * TODO:
   * 1. Проверка автоматического разжатия
   *  if (!manipulator.is_auto()) return;
   * TODO:
   * 2. Приём типа насадки (Щётка/ЕМА)
   * //Прописать в enum
   *  switch (manipulator.cap_type)
   *   case BRUSH:
   *     manipulator.force_needed = 100;
   *     manipulator.r0 = 347;
   *   case EMA:
   *     manipulator.force_needed = 150;
   *     manipulator.r0 = 331;
   * TODO:
   * 3. Проверка статуса блокировки
   *  if (manupulator.is_unlocked())
   *    shoulder.update_angle(shoulder.calc_angle());
   *    elbow.update_angle(elbow.calc_angle());
   *  else
   *    shoulder.update_angle();
   *    elbow.update_angle();
   * TODO:
   * 4. Обновить оставшиеся переменные
   *  shoulder.update_omega();
   *  elbow.update_omega();
   * TODO:
   * 5. Опкбликовать все переменные
   *  publish_all();
   */
}
void ManipulatorControlHandler::callback_manipulator(
    const std_msgs::Int64 &msg) {
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

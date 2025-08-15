#include "ShoulderHandler.h"
#include "Polynome.h"
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

#define SHOULDER_INPUT_NODE "/manipulator/shoulder_input"
#define SHOULDER_OUTPUT_NODE "/manipulator/shoulder_output"

const double ShoulderHandler::coeffs[]{-0.00011, 0.341, -110.2};

ShoulderHandler::ShoulderHandler(ros::NodeHandle *node) : pipeHandler(node) {
  subscriber = node->subscribe(SHOULDER_INPUT_NODE, 1000,
                               &ShoulderHandler::callback_shoulder, this);
  publisher = node->advertise<std_msgs::Int64>(SHOULDER_OUTPUT_NODE, 10);

  test_service = node->advertiseService(
      "/test_service", &ShoulderHandler::callback_service, this);
  //
  // TODO: Добавить обновление данных радиуса при инициализации
  //
}
void ShoulderHandler::callback_shoulder(const std_msgs::Int64 &msg) {
  // test_number += msg.data;
  // std_msgs::Int64 new_msg;
  // new_msg.data = test_number;
  // pub.publish(new_msg);
}
bool ShoulderHandler::callback_service(std_srvs::SetBool::Request &req,
                                       std_srvs::SetBool::Response &res) {
  // if (req.data) {

  //   test_number = 0;
  //   res.success = true;
  //   res.message = "Counter has been successfully reset";
  // } else {
  //   res.success = false;
  //   res.message = "Counter has not been reset";
  // }

  return true;
}
double ShoulderHandler::get_angle() {
  //
  // TODO: Добавить проверку на данные радиуса
  // Если (pipe.radius < H_min) -> обновить данные нв ноде PipeHandler
  //
  return Horner::polynome(coeffs, pipeHandler.get_radius());
}

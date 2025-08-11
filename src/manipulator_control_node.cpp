#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

int h_polynome(const std::vector<double> &coeff, double x) {
  if (coeff.empty())
    return 0;

  int result = coeff.back();
  for (size_t i = coeff.size() - 1; i > 0; --i)
    result = result * x + coeff[i - 1];
  return result;
}

struct pipe_t {
  uint16_t diameter;
  uint8_t thickness;
  double radius() const { return (double)diameter / 2.0 - thickness; }
};

class PipeHadler {

private:
  pipe_t params{};
  ros::Subscriber pipe_subscriber;

public:
  PipeHadler(ros::NodeHandle *node) {
    pipe_subscriber = node->subscribe("/pipe_subscriber_node", 1000,
                                      &PipeHadler::callback_pipe, this);
  }
  void callback_pipe(const std_msgs::Int64 &msg) {
    // params = pipe_t{msg->pipe_diam, msg->pipe_thickness};
    params =
        pipe_t{static_cast<uint16_t>(msg.data), static_cast<uint8_t>(msg.data)};
  }
  double get_radius() { return params.radius(); }
};

struct shoulder_t {};

class ShoulderHandler {

private:
  ros::Publisher shoulder_publisher;
  ros::Subscriber shoulder_subscriber;
  ros::ServiceServer test_service;
  PipeHadler pipeHadler;

public:
  ShoulderHandler(ros::NodeHandle *node) : pipeHadler(node) {
    shoulder_subscriber =
        node->subscribe("/manipulator/shoulder_input", 1000,
                        &ShoulderHandler::callback_shoulder, this);
    shoulder_publisher =
        node->advertise<std_msgs::Int64>("/manipulator/shoulder_output", 10);

    test_service = node->advertiseService(
        "/test_service", &ShoulderHandler::callback_service, this);
  }
  void callback_shoulder(const std_msgs::Int64 &msg) {
    /**
     *  TODO:
     *  Управление длиной:
     *  1.  Считывание сообщения с параметрами:
     *        F_mn, r0;
     *  2.  Задание ввода для:
     *        omega (w) = omega_input (w_input)
     *        D         = D_input,
     *        s         = s_input,
     *        r         = D / 2 - s;
     *  3.  Рассчитать shoulder_calc и elbow_calc;
     *  4.  Манипулятор зарблокирован?
     *        Да: присвоить углы из shoulder_calc и elbow_calc;
     *        Нет: присвоить углы из shoulder_input elbow_input;
     *  5.  Опубликовать значения:
     *        shoulder_angle_output               = shoulder_angle,
     *        elbow_angle_output                  = elbow_angle,
     *        omega_shoulder_output (w_s_output)  = omega (w);
     *        omega_elbow (w_e_output)            = omega (w);
     */

    // test_number += msg.data;
    // std_msgs::Int64 new_msg;
    // new_msg.data = test_number;
    // pub.publish(new_msg);
  }
  bool callback_service(std_srvs::SetBool::Request &req,
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
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle nh;
  ShoulderHandler nc = ShoulderHandler(&nh);
  ros::spin();
}

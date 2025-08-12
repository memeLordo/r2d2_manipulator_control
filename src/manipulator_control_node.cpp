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
  int test_number;
  ros::Publisher shoulder_publisher;
  ros::Subscriber shoulder_subscriber;
  ros::ServiceServer test_service;

public:
  ShoulderHandler(ros::NodeHandle *node) {
    test_number = 0;

    shoulder_subscriber =
        node->subscribe("/manipulator/shoulder_input", 1000,
                        &ShoulderHandler::callback_shoulder, this);
    shoulder_publisher =
        node->advertise<std_msgs::Int64>("/manipulator/shoulder_output", 10);

    test_service = node->advertiseService(
        "/test_service", &ShoulderHandler::callback_service, this);
  }
  void callback_shoulder(const std_msgs::Int64 &msg) {
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

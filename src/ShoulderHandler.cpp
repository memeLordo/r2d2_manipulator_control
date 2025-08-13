#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

struct shoulder_t {};

class ShoulderHandler {

private:
  int test_number;
  ros::Publisher shoulder_publisher;
  ros::Subscriber shoulder_subscriber;
  ros::ServiceServer test_service;
  const std::vector<double> coeffs{-0.00011, 0.341, -110.2};

public:
  ShoulderHandler(ros::NodeHandle *node) {
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
  double get_angle(double radius) { return h_polynome(coeffs, radius); }
};

#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <std_srvs/SetBool.h>

class NumberCounter {

private:
  int test_number;
  ros::Publisher pub;
  ros::Subscriber number_subscriber;
  ros::ServiceServer reset_service;

public:
  NumberCounter(ros::NodeHandle *node) {
    test_number = 0;

    pub = node->advertise<std_msgs::Int64>("/number_counter", 10);

    number_subscriber =
        node->subscribe("/number", 1000, &NumberCounter::callback_number, this);

    reset_service = node->advertiseService(
        "/reset_test_number", &NumberCounter::callback_reset_counter, this);
  }

  void callback_number(const std_msgs::Int64 &msg) {
    test_number += msg.data;
    std_msgs::Int64 new_msg;
    new_msg.data = test_number;
    pub.publish(new_msg);
  }

  bool callback_reset_counter(std_srvs::SetBool::Request &req,
                              std_srvs::SetBool::Response &res) {
    if (req.data) {
      test_number = 0;
      res.success = true;
      res.message = "Counter has been successfully reset";
    } else {
      res.success = false;
      res.message = "Counter has not been reset";
    }

    return true;
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle nh;
  NumberCounter nc = NumberCounter(&nh);
  ros::spin();
}

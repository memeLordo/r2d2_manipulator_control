#include "ManipulatorControl.hpp"
#include "ManipulatorService.hpp"
#include "TopicService.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  try {
    TopicServiceHandler ts(&node);
    ManipulatorControlHandler<> mc(&node);
    ManipulatorServiceHandler ms(&node, mc);
    ros::MultiThreadedSpinner spinner(8);
    ROS_INFO("Manipulator Control started");
    spinner.spin();
  } catch (std::exception &e) {
    ROS_ERROR_STREAM("Got exception: " << e.what());
  }
}

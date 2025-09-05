#include "ManipulatorControl.hpp"
#include "ManipulatorService.hpp"

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  // ros::AsyncSpinner spinner(2);
  // spinner.start();
  ManipulatorControlHandler<> mc(&node);
  ManipulatorServiceHandler ms(&node, &mc);
  ROS_INFO("Manipulator Control started");
  ros::spin();
  // ros::waitForShutdown();
}

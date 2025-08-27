#include "ManipulatorControl.h"
#include "ManipulatorService.h"
#include "utils/Debug.h"
#include <ros/node_handle.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  r2d2_debug::checkLogging(&node);
  // ros::AsyncSpinner spinner(2);
  // spinner.start();
  ManipulatorControlHandler<> mc(&node);
  ManipulatorServiceHandler ms(&node, mc);
  ROS_INFO("Manipulator Control started");
  ros::spin();
  // ros::waitForShutdown();
}

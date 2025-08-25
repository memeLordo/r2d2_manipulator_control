#include "ManipulatorControl.h"
#include "ManipulatorService.h"
#include <ros/node_handle.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;

  ros::AsyncSpinner spinner(2);
  spinner.start();
  ManipulatorServiceHandler ms(&node);
  ManipulatorControlHandler<> mc(&node);
  ros::waitForShutdown();
}

#include "ManipulatorControl.h"
#include "ros/init.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  ManipulatorControlHandler<> mc(&node);
  ros::spin();
}

#include "ManipulatorControl.h"
#include "ros/init.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  ros::Rate rate(10);
  ManipulatorControlHandler mc(&node);
  // ShoulderHandler nc = ShoulderHandler(&nh);
  while (ros::ok()) {
    mc.callback_manipulator();
    ros::spinOnce();
  }
  // ros::spin();
}

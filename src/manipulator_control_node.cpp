#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle nh;
  // ShoulderHandler nc = ShoulderHandler(&nh);
  ros::spin();
}

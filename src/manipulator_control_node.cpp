#include "ManipulatorControl.hpp"
#include "ManipulatorService.hpp"
#include "TopicService.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  try {
    CHECK_FOR_STACK_ERRORS();
    TopicServiceHandler ts{&node};
    ManipulatorControlHandler mc{&node};
    ManipulatorServiceHandler ms{&node, mc};
    ROS_INFO("Manipulator Control started");
    ros::spin();
  } catch (const r2d2_errors::RuntimeErrorStack& e) {
    ROS_ERROR_STREAM("Got exception: " << GREEN(e.what()));
    PROCESS_ERROR_STACK();
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Got exception: " << e.what());
  }
}

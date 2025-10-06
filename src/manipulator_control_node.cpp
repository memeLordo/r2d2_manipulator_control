#include "ManipulatorControl.hpp"
#include "ManipulatorService.hpp"
#include "TopicService.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  try {
    CHECK_FOR_ERROR_RECORD();
    TopicServiceHandler ts{&node};
    ManipulatorControlHandler mc{&node};
    ManipulatorServiceHandler ms{&node, mc};
    ROS_INFO("Manipulator Control started");
    ros::spin();
  } catch (const r2d2_errors::agent::RecordNotEmptyError& e) {
    ROS_ERROR_STREAM("Got exception: " << GREEN(e.what()));
    PROCESS_ERROR_RECORD();
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("Got exception: " << e.what());
  }
}

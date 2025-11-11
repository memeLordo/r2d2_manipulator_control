#include "ManipulatorControl.hpp"
#include "ManipulatorService.hpp"

/**
 * @brief Main entry point for the manipulator control node.
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * @return Exit code (0 on success)
 * @details Initializes ROS, creates manipulator control and service handlers,
 *          and enters the ROS spin loop. Handles exceptions and error records.
 */
int main(int argc, char** argv) {
  ros::init(argc, argv, "manipulator_control");
  ros::NodeHandle node;
  try {
    CHECK_FOR_ERROR_RECORD();
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

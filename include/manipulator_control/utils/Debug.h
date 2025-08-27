#ifndef PR_MANIPULATOR_CONTROL_DEBUG
#define PR_MANIPULATOR_CONTROL_DEBUG
#include <ros/node_handle.h>


namespace r2d2_debug {

inline void enableLogging() {
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();
}

inline void checkLogging(const ros::NodeHandle *node) {
  if (!node->hasParam("logger_level")) {
    ROS_WARN("Logger level not set");
    return;
  }
  std::string level{};
  node->param("logger_level", level);
  if (level != "DEBUG") {
    enableLogging();
  }
}

} // namespace r2d2_debug
#endif // !PR_MANIPULATOR_CONTROL_DEBUG

#ifndef PR_MANIPULATOR_CONTROL_DEBUG
#define PR_MANIPULATOR_CONTROL_DEBUG
#include "ros/node_handle.h"

namespace rkt_debug {

inline void enableLogging(ros::NodeHandle *node) {
  ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                 ros::console::levels::Debug);
  ros::console::notifyLoggerLevelsChanged();
}

inline void checkLogging(ros::NodeHandle *node) {
  std::string level;
  node->getParam("logger_level", level);
  if (level == "DEBUG") {
    enableLogging(node);
  }
}

} // namespace rkt_debug
#endif // !PR_MANIPULATOR_CONTROL_DEBUG

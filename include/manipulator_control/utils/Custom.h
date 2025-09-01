#ifndef PR_CUSTOM_H
#define PR_CUSTOM_H

#include "Debug.h"
#include <ros/console.h>

// // ROS debug function (requires ROS environment)
// #define ROS_DEBUG_FUNC(output, ...) \
//   do { \
//     std::ostringstream oss; \
//     oss << MAGENTA(__func__); \
//     debug_print_args(oss, #__VA_ARGS__, __VA_ARGS__); \
//     oss << " : " << WHITE(output); \
//     ROS_DEBUG_STREAM(oss.str()); \
//   } while (0)

#define ROS_DEBUG_FUNC(output, ...)                                            \
  LOG_FUNC_([](const std::string &msg) { ROS_DEBUG_STREAM(msg) }, output,      \
            __VA_ARGS__)

#endif // PR_CUSTOM_H

#ifndef INCLUDE_MANIPULATOR_CONTROL_TOPICAWAIT_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_TOPICAWAIT_HPP_

#include <ros/topic.h>

#include "r2d2_utils_pkg/Debug.hpp"

template <typename MsgType>
inline void waitForTopic(std::string_view name,
                         const std::string& outputTopic) {
  ROS_INFO_STREAM(CYAN("Waiting for " << name << " topic: " << outputTopic));
  ros::topic::waitForMessage<MsgType>(outputTopic);
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_TOPICAWAIT_HPP_

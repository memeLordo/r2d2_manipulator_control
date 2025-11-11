#ifndef INCLUDE_MANIPULATOR_CONTROL_TOPICAWAIT_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_TOPICAWAIT_HPP_

#include <ros/topic.h>

#include "r2d2_utils_pkg/Logging/Console.hpp"

/**
 * @brief Waits for a ROS topic to become available before proceeding.
 * @tparam MsgType The message type expected on the topic
 * @param name The name identifier for logging purposes
 * @param outputTopic The topic name to wait for
 * @details Blocks until the specified topic becomes available, useful for
 * ensuring topics are ready before subscribing or publishing.
 */
template <typename MsgType>
inline void waitForTopic(std::string_view name,
                         const std::string& outputTopic) {
  ROS_INFO_STREAM(CYAN("Waiting for " << name << " topic: " << outputTopic));
  ros::topic::waitForMessage<MsgType>(outputTopic);
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_TOPICAWAIT_HPP_

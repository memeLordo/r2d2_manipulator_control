#ifndef INCLUDE_MANIPULATOR_CONTROL_PAYLOADHANDLER_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_PAYLOADHANDLER_HPP_

#include <ros/topic.h>

#include "TopicAwait.hpp"
#include "r2d2_msg_pkg/DriverState.h"
#include "r2d2_utils_pkg/Debug.hpp"
#include "r2d2_utils_pkg/Math.hpp"
#include "r2d2_utils_pkg/Strings.hpp"
#include "r2d2_utils_pkg/Types.hpp"

class PayloadConfig {
 protected:
  const std::string m_name;
  const std::string m_outputTopic;

 protected:
  explicit PayloadConfig(std::string_view name = "payload")
      : m_name{r2d2_string::upper(name, 0, 1)},
        m_outputTopic{"/" + std::string{name} + "_output"} {};
};

template <typename T = double>
class PayloadHandler final : PayloadConfig {
 private:
  using PayloadConfig::m_name;
  using PayloadConfig::m_outputTopic;
  r2d2_type::callback::payload16_t m_callbackParams{};
  ros::Subscriber m_subscriber;

 public:
  PayloadHandler() = default;
  explicit PayloadHandler(ros::NodeHandle* node) : PayloadConfig{} {
    ROS_DEBUG_STREAM(MAGENTA("PayloadHandler()"));
    waitForTopic<r2d2_msg_pkg::DriverState>(m_name, m_outputTopic);
    m_subscriber = node->subscribe(m_outputTopic, 1,
                                   &PayloadHandler::callbackPayload, this);
  };
  ~PayloadHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~PayloadHandler()"));
    m_subscriber.shutdown();
  };

 private:
  void callbackPayload(const r2d2_msg_pkg::DriverStateConstPtr& msg) {
    m_callbackParams = r2d2_type::callback::payload16_t{msg->force};
  };

 public:
  [[nodiscard]] T getForce() const {
    const T force_{r2d2_process::Force::unwrap<T>(m_callbackParams.force)};
    ROS_DEBUG_STREAM(m_name << "::getForce() : " << WHITE(force_));
    return force_;
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_PAYLOADHANDLER_HPP_

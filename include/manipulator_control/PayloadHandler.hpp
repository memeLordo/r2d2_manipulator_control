#ifndef R2D2_PAYLOAD_HANDLER_HPP
#define R2D2_PAYLOAD_HANDLER_HPP

#include <ros/topic.h>

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
    waitForTopic();
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
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << m_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputTopic);
  };
  [[nodiscard]] T getForce() const {
    const T force_{r2d2_process::Force::unwrap<T>(m_callbackParams.force)};
    ROS_DEBUG_STREAM(m_name << "::getForce() : " << WHITE(force_));
    return force_;
  };
};
#endif  // R2D2_PAYLOAD_HANDLER_HPP

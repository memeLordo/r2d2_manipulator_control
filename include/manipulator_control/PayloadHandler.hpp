#ifndef R2D2_PAYLOAD_HANDLER_HPP
#define R2D2_PAYLOAD_HANDLER_HPP

#include <ros/topic.h>

#include "r2d2_msg_pkg/DriverState.h"
#include "r2d2_utils_pkg/Debug.hpp"
#include "r2d2_utils_pkg/Types.hpp"

template <typename T = double>
class PayloadHandler {
 private:
  static constexpr const char* s_name = "Payload";

 private:
  const std::string m_outputTopic;
  r2d2_type::callback::payload16_t m_callbackParams{};
  ros::Subscriber m_subscriber;

 public:
  PayloadHandler() = default;
  explicit PayloadHandler(ros::NodeHandle* node)
      : m_outputTopic{"/payload_output"} {
    waitForTopic();
    m_subscriber = node->subscribe(m_outputTopic, 1,
                                   &PayloadHandler::callbackPayload, this);
  };
  ~PayloadHandler() {
    ROS_DEBUG_STREAM(RED("~PayloadHandler()"));
    m_subscriber.shutdown();
  };

 private:
  void callbackPayload(const r2d2_msg_pkg::DriverStateConstPtr& msg) {
    m_callbackParams = r2d2_type::callback::payload16_t{msg->force};
  };

 public:
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << s_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputTopic);
  };
  T getForce() const {
    const T force_{static_cast<T>(m_callbackParams.force)};
    ROS_DEBUG_STREAM(s_name << "::getForce() : " << WHITE(force_));
    return force_;
  };
};
#endif  // PIPE_HANDLER_HPP

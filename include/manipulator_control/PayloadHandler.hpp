#ifndef R2D2_PAYLOAD_HANDLER_HPP
#define R2D2_PAYLOAD_HANDLER_HPP

#include "r2d2_msg_pkg/DriverState.h"
#include "utils/Debug.hpp"
#include "utils/Types.hpp"
#include <ros/topic.h>

template <typename T = double> class PayloadHandler {
private:
  static constexpr const char *s_name = "Payload";

  const std::string m_outputTopic;
  r2d2_type::payload16_t m_callbackParams{};
  ros::Subscriber m_subscriber;

public:
  PayloadHandler() = default;
  PayloadHandler(ros::NodeHandle *node);

private:
  void callbackPayload(const r2d2_msg_pkg::DriverStateConstPtr &msg) {
    m_callbackParams = r2d2_type::payload16_t{msg->force};
  };

public:
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << s_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputTopic);
  }
  T getForce() const {
    T force_ = static_cast<T>(m_callbackParams.force);
    ROS_DEBUG_STREAM(s_name << "::getForce() : " << WHITE(force_));
    return force_;
  };
};

#endif // PIPE_HANDLER_HPP

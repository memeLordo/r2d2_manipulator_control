#ifndef R2D2_PIPE_HANDLER_HPP
#define R2D2_PIPE_HANDLER_HPP

#include <ros/topic.h>

#include "r2d2_msg_pkg/PipeParameters.h"
#include "r2d2_utils_pkg/Debug.hpp"
#include "r2d2_utils_pkg/Strings.hpp"
#include "r2d2_utils_pkg/Types.hpp"

class PipeConfig {
 protected:
  const std::string m_name;
  const std::string m_outputTopic;

 protected:
  explicit PipeConfig(const std::string& name = "pipe")
      : m_name{r2d2_string::upper(name, 0, 1)},
        m_outputTopic{"/parameters/" + name} {};
};

template <typename T = double>
class PipeHandler : PipeConfig {
 private:
  using PipeConfig::m_name;
  using PipeConfig::m_outputTopic;
  r2d2_type::callback::pipe_t<T> m_callbackParams{};
  ros::Subscriber m_subscriber;

 public:
  PipeHandler() = default;
  explicit PipeHandler(ros::NodeHandle* node) : PipeConfig{} {
    waitForTopic();
    m_subscriber =
        node->subscribe(m_outputTopic, 1, &PipeHandler::callbackPipe, this);
  };
  ~PipeHandler() {
    ROS_DEBUG_STREAM(RED("~PipeHandler()"));
    m_subscriber.shutdown();
  };

 private:
  void callbackPipe(const r2d2_msg_pkg::PipeParametersConstPtr& msg) {
    m_callbackParams =
        r2d2_type::callback::pipe_t<T>{msg->pipe_diam, msg->pipe_thickness};
  };

 public:
  void waitForTopic() {
    ROS_INFO_STREAM(CYAN("Waiting for " << m_name << " topic..."));
    ros::topic::waitForMessage<r2d2_msg_pkg::PipeParameters>(m_outputTopic);
  };
  T getRadius() const {
    const T radius_{m_callbackParams.radius()};
    ROS_DEBUG_STREAM(m_name << "::getRadius() : " << WHITE(radius_));
    return radius_;
  };
};
#endif  // R2D2_PIPE_HANDLER_HPP

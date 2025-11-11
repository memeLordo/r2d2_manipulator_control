#ifndef INCLUDE_MANIPULATOR_CONTROL_PIPEHANDLER_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_PIPEHANDLER_HPP_

#include <ros/topic.h>

#include "TopicService.hpp"
#include "r2d2_msg_pkg/PipeParameters.h"
#include "r2d2_utils_pkg/Logging/Console.hpp"
#include "r2d2_utils_pkg/Logging/Custom.hpp"
#include "r2d2_utils_pkg/Strings.hpp"
#include "r2d2_utils_pkg/Types.hpp"

class PipeConfig {
 protected:
  const std::string m_name;
  const std::string m_outputTopic;

 protected:
  /**
   * @brief Constructs a PipeConfig object with the specified name.
   * @param name The name of the pipe (default: "pipe")
   * @details Initializes the pipe name and output topic name for parameters.
   */
  explicit PipeConfig(std::string_view name = "pipe")
      : m_name{r2d2_string::upper(name, 0, 1)},
        m_outputTopic{"/parameters/" + std::string{name}} {};
};

template <typename T = double>
class PipeHandler final : PipeConfig {
 private:
  using PipeConfig::m_name;
  using PipeConfig::m_outputTopic;
  r2d2_type::callback::pipe_t<T> m_callbackParams{};
  ros::Subscriber m_subscriber;

 public:
  PipeHandler() = default;
  /**
   * @brief Constructs a PipeHandler and initializes ROS subscriber and
   * parameter service.
   * @param node Pointer to the ROS node handle
   * @details Subscribes to pipe parameters topic and calls the parameter update
   * service.
   */
  explicit PipeHandler(ros::NodeHandle* node) : PipeConfig{} {
    ROS_DEBUG_STREAM(MAGENTA(m_name + "Handler()"));
    m_subscriber =
        node->subscribe(m_outputTopic, 1, &PipeHandler::callbackPipe, this);
    TopicServiceHandler ts{node};
  };
  ~PipeHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~" + m_name + "Handler()"));
    m_subscriber.shutdown();
  };

 private:
  /**
   * @brief Callback function for receiving pipe parameter messages.
   * @param msg The pipe parameters message containing diameter and thickness
   * @details Updates the internal callback parameters with the latest pipe
   * dimensions.
   */
  void callbackPipe(const r2d2_msg_pkg::PipeParametersConstPtr& msg) {
    m_callbackParams =
        r2d2_type::callback::pipe_t<T>{msg->pipe_diam, msg->pipe_thickness};
  };

 public:
  /**
   * @brief Calculates the pipe radius from diameter and thickness.
   * @return The calculated pipe radius value
   */
  [[nodiscard]] T getRadius() const {
    const T radius_{m_callbackParams.radius()};
    ROS_DEBUG_NAMED_FUNC_C(m_name, radius_, "");
    return radius_;
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_PIPEHANDLER_HPP_

#ifndef INCLUDE_MANIPULATOR_CONTROL_PAYLOADHANDLER_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_PAYLOADHANDLER_HPP_

#include <ros/topic.h>

#include "TopicAwait.hpp"
#include "r2d2_msg_pkg/DriverState.h"
#include "r2d2_utils_pkg/Logging/Console.hpp"
#include "r2d2_utils_pkg/Logging/Custom.hpp"
#include "r2d2_utils_pkg/Math.hpp"
#include "r2d2_utils_pkg/Strings.hpp"
#include "r2d2_utils_pkg/Types.hpp"

class PayloadConfig {
 protected:
  const std::string m_name;
  const std::string m_outputTopic;

 protected:
  /**
   * @brief   Constructs a PayloadConfig object with the specified name.
   *
   * @param   name The name of the payload (default: "payload")
   *
   * @details Initializes the payload name and output topic name.
   */
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
  volatile bool m_needsControl{true};

 public:
  PayloadHandler() = default;

  /**
   * @brief   Constructs a PayloadHandler and initializes ROS subscriber.
   *
   * @param   node Pointer to the ROS node handle
   *
   * @details Waits for the driver state topic to become available, then
   *          subscribes to it.
   */
  explicit PayloadHandler(ros::NodeHandle* node) : PayloadConfig{} {
    ROS_DEBUG_STREAM(MAGENTA(m_name + "Handler()"));
    waitForTopic<r2d2_msg_pkg::DriverState>(m_name, m_outputTopic);
    m_subscriber = node->subscribe(m_outputTopic, 1,
                                   &PayloadHandler::callbackPayload, this);
  };
  ~PayloadHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~" + m_name + "Handler()"));
    m_subscriber.shutdown();
  };

 private:
  /**
   * @brief   Callback function for receiving payload driver state messages.
   *
   * @param   msg The driver state message containing force data
   *
   * @details Updates the internal callback parameters with the latest force
   *            value.
   */
  void callbackPayload(const r2d2_msg_pkg::DriverStateConstPtr& msg) {
    m_callbackParams = r2d2_type::callback::payload16_t{msg->force};
  };

 public:
  /**
   * @brief   Sets the control flag to the specified value.
   *
   * @param   needsControl Boolean indicating whether control is needed
   */
  void setControl(const bool needsControl) { m_needsControl = needsControl; };

  /**
   * @brief   Resets the control flag to false.
   */
  void resetControl() { setControl(false); };

  /**
   * @brief   Checks if the payload needs control.
   *
   * @return  True if control is needed, false otherwise
   */
  [[nodiscard]] bool needsControl() const {
    ROS_DEBUG_NAMED_COLORED_VARS_C(m_name, ANSI_CYAN, m_needsControl);
    return m_needsControl;
  };

  /**
   * @brief   Gets the current force value from the callback data.
   *
   * @return  The unwrapped force value from the latest driver state message
   */
  [[nodiscard]] T getForce() const {
    const T force_{r2d2_process::Force::unwrap<T>(m_callbackParams.force)};
    ROS_DEBUG_NAMED_FUNC_C(m_name, force_, "");
    return force_;
  };
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_PAYLOADHANDLER_HPP_

#ifndef INCLUDE_MANIPULATOR_CONTROL_TOPICSERVICE_HPP_
#define INCLUDE_MANIPULATOR_CONTROL_TOPICSERVICE_HPP_

#include <ros/node_handle.h>

#include "r2d2_msg_pkg/GetParams.h"
#include "r2d2_utils_pkg/Logging/Console.hpp"

class TopicServiceHandler final {
 private:
  ros::ServiceClient m_client;

 public:
  TopicServiceHandler() = default;
  /**
   * @brief Constructs a TopicServiceHandler and calls the parameter update
   * service.
   * @param node Pointer to the ROS node handle
   * @details Connects to the /get_params service and requests a parameter
   * update. Logs a warning if the service call fails.
   */
  explicit TopicServiceHandler(ros::NodeHandle* node) noexcept {
    ROS_DEBUG_STREAM(MAGENTA("TopicServiceHandler()"));
    using r2d2_msg_pkg::GetParams;
    m_client = node->serviceClient<GetParams>("/get_params");

    GetParams srv_;
    srv_.request.update = true;

    if (!(m_client.exists() && m_client.call(srv_))) {
      ROS_WARN("Couldn't update params!");
      return;
    }
    ROS_INFO_STREAM(CYAN("Params updated succsessfully!"));
  };
  /**
   * @brief Destructor for TopicServiceHandler.
   */
  ~TopicServiceHandler() noexcept {
    ROS_DEBUG_STREAM(RED("~TopicServiceHandler()"));
  }
};
#endif  // INCLUDE_MANIPULATOR_CONTROL_TOPICSERVICE_HPP_

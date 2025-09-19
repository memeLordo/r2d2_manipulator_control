#ifndef R2D2_TOPIC_SERVICE_HANDLER_HPP
#define R2D2_TOPIC_SERVICE_HANDLER_HPP

#include "r2d2_msg_pkg/GetParams.h"
#include "ros/node_handle.h"
#include "utils/Debug.hpp"

class TopicServiceHandler {
 private:
  ros::ServiceClient m_client;

 public:
  TopicServiceHandler() = default;
  explicit TopicServiceHandler(ros::NodeHandle *node) {
    using r2d2_msg_pkg::GetParams;

    m_client = node->serviceClient<GetParams>("/get_params");

    GetParams srv_;
    srv_.request.update = true;

    if (!(m_client.exists() && m_client.call(srv_))) {
      ROS_ERROR("Couldn't update params!");
      return;
    }
    ROS_INFO_STREAM(CYAN("Params updated succsessfully!"));
  };
};
#endif  // R2D2_TOPIC_SERVICE_HANDLER_HPP

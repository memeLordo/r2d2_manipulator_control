#include "PayloadHandler.hpp"
#include <ros/node_handle.h>
#include <ros/topic.h>

template <typename T>
PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node)
    : m_outputNode{"/payload_output"} {
  constexpr int QUEUE_SIZE = 8;
  ROS_INFO_STREAM(CYAN("Waiting for " << s_name << " topic..."));
  ros::topic::waitForMessage<r2d2_msg_pkg::DriverState>(m_outputNode);
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &PayloadHandler::callbackPayload, this);
}
template class PayloadHandler<>;

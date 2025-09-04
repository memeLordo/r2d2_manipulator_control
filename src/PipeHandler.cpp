#include "PipeHandler.hpp"
#include "r2d2_msg_pkg/PipeParameters.h"
#include <ros/node_handle.h>
#include <ros/topic.h>

template <typename T>
PipeHandler<T>::PipeHandler(ros::NodeHandle *node)
    : m_outputNode{"/parameters/pipe"} {
  constexpr int QUEUE_SIZE = 8;
  ROS_INFO_STREAM(CYAN("Waiting for " << s_name << " topic..."));
  ros::topic::waitForMessage<r2d2_msg_pkg::PipeParameters>(m_outputNode);
  m_subscriber = node->subscribe(m_outputNode, QUEUE_SIZE,
                                 &PipeHandler::callbackPipe, this);
}
template class PipeHandler<>;

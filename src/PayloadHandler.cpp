#include "PayloadHandler.hpp"

template <typename T>
PayloadHandler<T>::PayloadHandler(ros::NodeHandle *node)
    : m_outputTopic{"/payload_output"} {
  waitForTopic();
  m_subscriber = node->subscribe(m_outputTopic, 10,
                                 &PayloadHandler::callbackPayload, this);
}
template class PayloadHandler<>;

#include <ros/ros.h>
#include <std_msgs/Int64.h>

struct pipe_t {
  uint16_t diameter;
  uint8_t thickness;
  double radius() const { return (double)diameter / 2.0 - thickness; }
};

class PipeHadler {

private:
  pipe_t params{};
  ros::Subscriber pipe_subscriber;

public:
  PipeHadler(ros::NodeHandle *node) {
    pipe_subscriber = node->subscribe("/pipe_subscriber_node", 1000,
                                      &PipeHadler::callback_pipe, this);
  }
  void callback_pipe(const std_msgs::Int64 &msg) {
    // params = pipe_t{msg->pipe_diam, msg->pipe_thickness};
    params =
        pipe_t{static_cast<uint16_t>(msg.data), static_cast<uint8_t>(msg.data)};
  }
  // double get_radius() { return params.radius(); }
};

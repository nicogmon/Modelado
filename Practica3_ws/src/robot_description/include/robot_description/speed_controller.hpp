#ifndef ROBOT_DESCRIPTION__SPEED_CONTROLLER_HPP_
#define ROBOT_DESCRIPTION__SPEED_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>

namespace robot_description
{
class  SpeedControllerNode: public rclcpp::Node
{

public:
  SpeedControllerNode();
  

private:
  void timer_callback();
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::Twist msg_;
  std::chrono::steady_clock::time_point start_time_;
  uint64_t count_;
  float vel_;
};
}  // namespace robot_description

#endif  // ROBOT_DESCRIPTION__SPEED_CONTROLLER_HPP_
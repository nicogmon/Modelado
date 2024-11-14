#include "robot_description/speed_controller.hpp"
using namespace std::literals::chrono_literals;





namespace robot_description
{
SpeedControllerNode::SpeedControllerNode()
: Node("speed_controller_node")
{
  vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/rover_base_control/cmd_vel_unstamped", 10);
  timer_ = this->create_wall_timer(25ms, std::bind(&SpeedControllerNode::timer_callback, this));
  vel_ = 0.0;
  
  start_time_ = std::chrono::steady_clock::now();
  
  // la velocidad tiene que variar para seguir la grafica del enunciado sin contar con el choque
}

void 
SpeedControllerNode::timer_callback()
{
  auto current_time = std::chrono::steady_clock::now();
  auto elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time_).count();
  // RCLCPP_INFO(get_logger(), "Elapsed time: %ld seconds", elapsed_seconds);
  if (elapsed_seconds < 5 )
  {
    vel_ += 0.025;
    // RCLCPP_INFO(get_logger(), "Aumentando Vel: %f", vel_);
  }else if (elapsed_seconds > 4.9 && elapsed_seconds < 10)
  {  
    vel_ = 5;
    // RCLCPP_INFO(get_logger(), "Manteniendo Vel: %f", vel_);
  }else if (elapsed_seconds > 9.9 && elapsed_seconds < 15)
  {
    vel_ -= 0.025;
  }else if (elapsed_seconds > 14.9)
  {
    vel_ = 0;
    
  }
  RCLCPP_INFO(get_logger(), "Vel: %f", vel_);
  msg_.linear.x = vel_;
  vel_pub_->publish(msg_);
  if (vel_ == 0){
    RCLCPP_INFO(get_logger(), "Velocidad 0, terminando ejecucion");
    rclcpp::shutdown();
  
  }
}
} // namespace vel_pub_sub


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<robot_description::SpeedControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
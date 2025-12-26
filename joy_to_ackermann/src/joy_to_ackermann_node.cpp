#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "joy_to_ackermann/joy_to_ackermann.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<joy_to_ackermann::JoyToAckermann>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}

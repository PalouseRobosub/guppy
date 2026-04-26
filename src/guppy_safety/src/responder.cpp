#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "guppy_msgs/msg/safety_level.hpp"

// Responds to changes in the safety level, hooray.
class Responder : public rclcpp::Node
{
public:
  Responder()
  : Node("safety_responder")
  {
    // Fault state already set somewhere else
    auto respond_to_change  = [this](guppy_msgs::msg::SafetyLevel::UniquePtr msg) -> void { 
      //RCLCPP_INFO(this->get_logger(), "[state] '%s'", msg->data.c_str());
      printf("state chnaged"); 
    };

    sub_level = this->create_subscription<guppy_msgs::msg::SafetyLevel>("safety_levelevel", 10, respond_to_change);
  };

private:
  // All subsciption types
  rclcpp::Subscription<guppy_msgs::msg::SafetyLevel>::SharedPtr sub_level;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Responder>());
  rclcpp::shutdown();
  return 0;
}

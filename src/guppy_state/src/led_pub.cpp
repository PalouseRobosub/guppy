#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "guppy_msgs/srv/send_can.hpp"

class LEDStatePublisher : public rclcpp::Node
{
public:
  LEDStatePublisher() : Node("led_state")
  {
    auto topic_callback =
      [this](std_msgs::msg::UInt8::UniquePtr msg) -> void {
        auto request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
        request->id = 0x201;
        std::vector<uint8_t> bytes = { msg->data };
        request->data = bytes;
        client_->async_send_request(request);
      };

   
    subscription_ = this->create_subscription<std_msgs::msg::UInt8>("state", 10, topic_callback);
    client_ = this->create_client<guppy_msgs::srv::SendCan>("can_tx");
  }
private:
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDStatePublisher>());
  rclcpp::shutdown();
  return 0;
}

  

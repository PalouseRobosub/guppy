#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "guppy_msgs/srv/send_can.hpp"

class LEDStatePublisher : public rclcpp::Node
{
public:
  LEDStatePublisher() : Node("led_pub")
  {
    auto topic_callback =
      [this](std_msgs::msg::UInt8::UniquePtr msg) -> void {
        current_state = msg->data;
      };

    subscription_ = this->create_subscription<std_msgs::msg::UInt8>("state", 10, topic_callback);
    client_ = this->create_client<guppy_msgs::srv::SendCan>("can_tx");


    timer = this->create_wall_timer(
      std::chrono::milliseconds(100),
      [this](){
        auto request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
        request->id = 0x201;
        request->data = { current_state };
        client_->async_send_request(request);
      
      }
    );
  }
private:
  uint8_t current_state = 0;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr subscription_;
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LEDStatePublisher>());
  rclcpp::shutdown();
  return 0;
}

  

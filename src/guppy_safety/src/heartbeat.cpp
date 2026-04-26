#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "guppy_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Checks if cmd_vel is stale, if it is will publish a disabled state
class Heartbeat : public rclcpp::Node
{
public:
  Heartbeat()
  : Node("safety_heartbeat")
  {
    rclcpp::QoS watchedQos(1);
    watchedQos.deadline(rclcpp::Duration::from_seconds(0.5));
    # ^ maybe add qos to the other cmd_vel publishers?
    
    rclcpp::SubscriptionOptions watchedOptions;
    watchedOptions.event_callbacks.deadline_callback = panic;

    pub_state = this->create_publisher<guppy_msgs::msg::State>("State", 10);
    sub_cmdvel  = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, 
        watchedQos, empty_callback, watchedOptions);
  };

private:
  rclcpp::Publisher<guppy_msgs::msg::State>::SharedPtr pub_state;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
  
  void empty_callback() {};

  void panic() {
        auto message = guppy_msgs::msg::State();
        message.state = 5; // 5 = DISABLED, 6 = FAULT 
      
	      RCLCPP_INFO(this->get_logger(), "cmd_vel detected to be stale";
        this->pub_state->publish(message);
  };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Heartbeat>());
  rclcpp::shutdown();
  return 0;
}

#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "guppy_msgs/msg/safety_level.hpp"

#include "std_msgs/msg/string.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"

// Checks a bunch of topics, and sets a safety level
// if one or more of them shows an unhealthy value
class Checker : public rclcpp::Node
{
public:
  Checker()
  : Node("safety_checker")
  {
    // Publish result of check as a level
    pub_level = this->create_publisher<guppy_msgs::msg::SafetyLevel>("safety_level", 10);
  
    // Publish a safe level to prevent the end of the world [probably]
    current_level = 2;
    publish_level(current_level);

    // CAN RX timeout
    /* auto check_CAN    = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "[can_rx] '%s'", msg->data.c_str());
        update_status(5);
        printf("can_rx ran");
    }; */

    // Impossible Odom
    auto check_Odom   = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void {
      //RCLCPP_INFO(this->get_logger(), "[odom] '%s'", msg->data.c_str());
      printf("odom ran");
      compare_level(5);
    };

    // Excessive Commanded Velocity 
    auto check_CmdVel = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void {
      //RCLCPP_INFO(this->get_logger(), "[cmd_vel] '%s'", msg->data.c_str());
      printf("cmdvel ran");
      compare_level(7); 
    };

    // Fault state already set somewhere else
    auto check_state  = [this](guppy_msgs::msg::State::UniquePtr msg) -> void { 
      //RCLCPP_INFO(this->get_logger(), "[state] '%s'", msg->data.c_str());
      printf("state ran");
      compare_level(6); 
    };

    // Subscibe to all checked topics
    //sub_canrx  = this->create_subscription<std_msgs::msg::String>(     "can_rx",  10, check_CAN);
    sub_odom   = this->create_subscription<nav_msgs::msg::Odometry>(   "odom",    10, check_Odom);
    sub_cmdvel = this->create_subscription<geometry_msgs::msg::Twist>( "cmd_vel", 10, check_CmdVel);
    sub_state  = this->create_subscription<guppy_msgs::msg::State>(    "state",   10, check_state);
  };

private:
  // Store current 
  int current_level;

  // Publisher
  rclcpp::Publisher<guppy_msgs::msg::SafetyLevel>::SharedPtr pub_level;

  // All subsciption types
  //rclcpp::Subscription<guppy_msgs::msg::can_frame>::SharedPtr sub_canrx;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
  rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr sub_state;
 
  // Publish the current level 
  // (To be called in each checker callback)
  void compare_level(int new_level) {
    if (new_level > current_level) {
        publish_level(new_level);
    }; 
  };

  void publish_level(int new_level) {
        current_level = new_level;

        auto message = guppy_msgs::msg::SafetyLevel();
        message.level = current_level;
      
	      //RCLCPP_INFO(this->get_logger(), "[Published Level] '%s'", message.data.c_str());
        this->pub_level->publish(message);
  };
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Checker>());
  rclcpp::shutdown();
  return 0;
}

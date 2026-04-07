#include <memory>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "guppy_safety/msg/Status.hpp"

#include "std_msgs/msg/string.hpp"
#include "guppy_msgs/msg/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/vector3.hpp"
//#include "nav_msgs/msg/Odometry.hpp"

// Checks a bunch of topics, and sets a safety level
// if one or more of them shows an unhealthy value
class Checker : public rclcpp::Node
{
public:
  Checker()
  : Node("safety_checker")
  {
    // Publish result of check as a level
    pub_level = this->create_publisher<guppy_safety::msg::Status>("safety_level", 10);
  
    current_level = 2;
    publish_status(5);

    // CAN RX timeout
    /* auto check_CAN    = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "[can_rx] '%s'", msg->data.c_str());
        update_status(5);
        printf("can_rx ran");
    }; */

    // Impossible Odom
    /* auto check_Odom   = [this](nav_msgs::msg::Odometry::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "[odom] '%s'", msg->data.c_str());
         printf("odom ran");    update_status(5); 
    }; */

    // Excessive Commaned Velocity 
    auto check_CmdVel = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void {
      //RCLCPP_INFO(this->get_logger(), "[cmd_vel] '%s'", msg->data.c_str());
      update_status(7); 
    };

    // Fault state already set somewhere else
    auto check_state  = [this](guppy_msgs::msg::State::UniquePtr msg) -> void { 
      //RCLCPP_INFO(this->get_logger(), "[state] '%s'", msg->data.c_str());
      update_status(6); 
    };

    // Subscibe to all checked topics
    //sub_canrx  = this->create_subscription<std_msgs::msg::String>(     "can_rx",  10, check_CAN);
    //sub_odom   = this->create_subscription<nav_msgs::msg::Odometry>(   "odom",    10, check_Odom);
    sub_cmdvel = this->create_subscription<geometry_msgs::msg::Twist>( "cmd_vel", 10, check_CmdVel);
    sub_state  = this->create_subscription<guppy_msgs::msg::State>(    "state",   10, check_state);
  };

private:
  // Store current 
  int current_level;

  // Publisher
  rclcpp::Publisher<guppy_safety::msg::Status>::SharedPtr pub_level;

  // All subsciption types
  //rclcpp::Subscription<guppy_msgs::msg::can_frame>::SharedPtr sub_canrx;
  //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmdvel;
  rclcpp::Subscription<guppy_msgs::msg::State>::SharedPtr sub_state;
 
  // Publish the current level 
  // (To be called in each checker callback)
  void update_status(int new_level) {
    // Check if new is more severe than old
    if (new_level > current_level) {
        publish_status(new_level);
    }; 
  };

  void publish_status(int new_level) {
        // Move the the new value into current
        current_level = new_level;

        // Create a message
        auto message = guppy_safety::msg::Status();
        message.level = current_level;
      
        // Publish that new level
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

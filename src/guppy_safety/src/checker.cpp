#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// Checks a bunch of topics, and sets a status 
// if one or more of them shows an unhealthy value
class Checker : public rclcpp::Node
{
public:
  Checker()
  : Node("safety_checker")
  {
    // Publish result of check as a status
    //auto updateStatus = {

    //};

    // v Check topic callbacks v

    // CAN RX timeout
    auto check_CAN    = [this](std_msgs::msg::String::UniquePtr msg) -> void {
        RCLCPP_INFO(this->get_logger(), "[can_rx] '%s'", msg->data.c_str());
    };

    // Impossible Odom
    auto check_Odom   = [this](std_msgs::msg::String::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "[odom] '%s'", msg->data.c_str());
    };

    // Excessive Commaned Velocity 
    auto check_CmdVel = [this](std_msgs::msg::String::UniquePtr msg) -> void {
      RCLCPP_INFO(this->get_logger(), "[cmd_vel] '%s'", msg->data.c_str());  
    };

    // Fault state already set somewhere else
    auto check_state  = [this](std_msgs::msg::String::UniquePtr msg) -> void { 
      RCLCPP_INFO(this->get_logger(), "[state] '%s'", msg->data.c_str());
    };

    // Subscibe to all checked topics
    sub_canrx  = this->create_subscription<std_msgs::msg::String>("can_rx",  10, check_CAN);
    sub_odom   = this->create_subscription<std_msgs::msg::String>("odom",    10, check_Odom);
    sub_cmdvel = this->create_subscription<std_msgs::msg::String>("cmd_vel", 10, check_CmdVel);
    sub_state  = this->create_subscription<std_msgs::msg::String>("state",   10, check_state);
  }

private:
  // Status
  int status = 5;

  // All subsciption types
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_canrx;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_odom;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_cmdvel;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_state;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Checker>());
  rclcpp::shutdown();
  return 0;
}

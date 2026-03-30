#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Checker : public rclcpp::Node
{

    bool status = 4; // 4 is Good
// Variables holding each thing here

public:
  Checker()
  : Node("safety_checker")
  {
    // Current status variable

    // Create status publiser
    publisher_ = this->create_publisher<std_msgs::msg::String>("safety_status", 10);
 
    // Push new status varible to ros
    void update_status() {
      self.publisher_.publish(status);
    }

    // Check if the barometer data is valid
    auto check_barometer =
      [this](std_msgs::msg::String::UniquePtr msg) -> void {
        char[] data = msg->data.c_str();
        RCLCPP_INFO(this->get_logger(), "Barometer: '%s'", data);
        // ^ might want to remove this?

        // Check if value is invalid
        if (data >= 1000 && data <= 0) {
          RCLCPP_WARN(this->get_logger(), "Barometer: '%s'", data);
          status = 3;
          update_status();
        }
      };

    // Do the above for all checked systems
    

    subscription_ =
      this->create_subscription<std_msgs::msg::String>("topic", 10, check_barometer);
      // Do the above for all checked systems
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Checker>());
  rclcpp::shutdown();

  return 0;
}

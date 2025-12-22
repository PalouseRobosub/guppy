#include <chrono>
#include <iostream>
#include <limits>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;

class ThrottlePublisher : public rclcpp::Node  // inherits from standard ros node class
{
 public:
  ThrottlePublisher() : Node("throttle_publisher") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32>("/can/id101", 10);                 // creates a publisher for the "throttle" topic
    timer_ = this->create_wall_timer(500ms, std::bind(&ThrottlePublisher::timer_callback, this));  // runs timer_callback every 500ms
  }

 private:
  void timer_callback() {
    static bool waiting_for_input = false;
    static float user_value{0.0f};

    if (!waiting_for_input) {
      std::cout << "Enter throttle value [-1.0, 1.0]: ";
      std::cin >> user_value;  // BLOCKING

      // validate input
      if (std::cin.fail() || user_value < -1.0f || user_value > 1.0f) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Invalid input. Using last valid value: " << user_value << std::endl;
      } else {
        waiting_for_input = true;
      }
    }

    if (waiting_for_input) {
      auto msg = std_msgs::msg::Float32();  // creates a new message
      msg.data = user_value;                // sets the message data
      publisher_->publish(msg);             // publishes the message
      RCLCPP_INFO(this->get_logger(), "Published throttle: %f", static_cast<double>(user_value));
      waiting_for_input = false;  // reset for next input
    }
  }

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ThrottlePublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* an enumeration to hold the current state of guppy */
typedef enum guppy_state_ {
  /* guppy is navigating and being controlled by nav2 */
  NAV2 = 0,

  /* guppy is being controlled by a task node */
  TASK = 1,

  /* guppy is being controlled by teleop */
  TELEOP = 2,

  /* guppy is holding position and orientation in water */
  HOLDING = 3,

  /* motors are disabled */
  DISABLED = 4,

  /* motors are disabled and there is a fault */
  FAULT = 5
} GuppyState;

/* stores the state names for publishing */
string STATE_STRINGS[] = {"NAV2", "TASK", "TELEOP", "HOLDING", "DISABLED", "FAULT"};

/* a state machine node that handles the state changes of guppy */
class StateMachine : public rclcpp::Node
{
public:
  StateMachine() : Node("state_machine") {
    state_publisher_ = this->create_publisher<std_msgs::msg::String>("/state", 10);
    timer_ = this->create_wall_timer(500ms, publish_callback);
  }

  void publish_callback() {
    auto message = std_msgs::msg::String();
    message.data = STATE_STRINGS[state_];
    state_publisher_.publish(message);
  }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    GuppyState state_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateMachine>());
  rclcpp::shutdown();
  return 0;
}
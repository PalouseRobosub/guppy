// WRITTEN BY CHATGPT FOR TESTING

#include "rclcpp/rclcpp.hpp"
#include "guppy_msgs/srv/send_can.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>
#include <vector>

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  if (argc < 3) {
    RCLCPP_INFO(
      rclcpp::get_logger("can_tx_client"),
      "Usage: can_tx_client <can_id> <byte0> [byte1 byte2 ...]"
    );
    return 1;
  }

  auto node = rclcpp::Node::make_shared("can_tx_client");
  auto client = node->create_client<guppy_msgs::srv::SendCan>("can_tx");

  // Wait for service
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for service.");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "Waiting for can_tx service...");
  }

  auto request = std::make_shared<guppy_msgs::srv::SendCan::Request>();

  // CAN ID
  request->id = static_cast<uint32_t>(std::stoul(argv[1], nullptr, 0));

  // Data bytes
  for (int i = 2; i < argc; ++i) {
    request->data.push_back(
      static_cast<uint8_t>(std::stoul(argv[i], nullptr, 0))
    );
  }

  auto future = client->async_send_request(request);

  // Block until response
  auto result = rclcpp::spin_until_future_complete(node, future);

  if (result == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO(
      node->get_logger(),
      "CAN TX %s",
      response->written != -1 ? "SUCCESS" : "FAILED"
    );
  } else {
    RCLCPP_ERROR(node->get_logger(), "Service call failed");
  }

  rclcpp::shutdown();
  return 0;
}

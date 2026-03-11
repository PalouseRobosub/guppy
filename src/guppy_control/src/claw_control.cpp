#include "guppy_msgs/srv/send_claw.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <cstring>
#include <memory>

#include <net/if.h>
#include <unistd.h>

void send(const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request, 
  std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response,
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client,
  std::shared_ptr<rclcpp::Node> node
) {

    if (request->degrees < 0.0 || request->degrees > 180.0) {
      RCLCPP_WARN(node->get_logger(), "Degrees out of range: %f", request->degrees);
      response->success = false;
      return;
    }

    auto can_request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
    can_request->id = 0x000;

    can_request->data.resize(sizeof(double));
    std::memcpy(
        can_request->data.data(),
        &request->degrees,
        sizeof(double)
    );

    auto result = client->async_send_request(can_request);

    if (result.wait_for(std::chrono::seconds(2)) == std::future_status::ready)
    {
      RCLCPP_INFO(node->get_logger(), "CAN call succeeded");
    }
    else
    {
      RCLCPP_ERROR(node->get_logger(), "CAN call timed out");
    }

    RCLCPP_INFO(node->get_logger(), "Received degrees: %f", request->degrees);
    response->success = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("claw_service");

  // --- FIX: Two separate callback groups ---
  // The service callback blocks, so it needs its own MutuallyExclusive group.
  // The client needs a separate group so its response can be processed
  // by another thread while the service callback is blocking.
  auto service_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);
  auto client_cbg = node->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client =
    node->create_client<guppy_msgs::srv::SendCan>(
      "can_tx",
      rmw_qos_profile_services_default,
      client_cbg);  // <-- assign client to its own group

  auto service = node->create_service<guppy_msgs::srv::SendClaw>(
      "claw_tx", 
      [client, node](
        const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request,
        std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response)
          {
             send(request, response, client, node); 
          },
      rmw_qos_profile_services_default,
      service_cbg);  // <-- assign service to its own group

  RCLCPP_INFO(node->get_logger(), "CLAW TX service ready");

  // --- FIX: MultiThreadedExecutor so both groups can run concurrently ---
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}git add .
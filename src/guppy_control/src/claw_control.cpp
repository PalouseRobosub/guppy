#include "guppy_msgs/srv/send_claw.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <cstring>
#include <memory>
#include <net/if.h>
#include <unistd.h>

class ClawService : public rclcpp::Node
{
public:
  ClawService() : Node("claw_service")
  {
    service_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    client_ = this->create_client<guppy_msgs::srv::SendCan>(
      "can_tx",
      rmw_qos_profile_services_default,
      client_cbg_);

    service_ = this->create_service<guppy_msgs::srv::SendClaw>(
      "claw_tx",
      [this](
        const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request,
        std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response)
      {
        this->send(request, response);
      },
      rmw_qos_profile_services_default,
      service_cbg_);

    RCLCPP_INFO(this->get_logger(), "CLAW TX service ready");
  }

private:
  rclcpp::CallbackGroup::SharedPtr service_cbg_;
  rclcpp::CallbackGroup::SharedPtr client_cbg_;
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client_;
  rclcpp::Service<guppy_msgs::srv::SendClaw>::SharedPtr service_;

  void send(
    const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request,
    std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response)
  {
    if (request->degrees < 0.0 || request->degrees > 180.0) {
      RCLCPP_WARN(this->get_logger(), "Degrees out of range: %f", request->degrees);
      response->success = false;
      return;
    }

    auto can_request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
    can_request->id = 0x41A;
    can_request->data.resize(sizeof(double));
    std::memcpy(
      can_request->data.data(),
      &request->degrees,
      sizeof(double));

    auto result = client_->async_send_request(can_request);
    if (result.wait_for(std::chrono::seconds(2)) == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "CAN call succeeded");
      response->success = true;
    } else {
      RCLCPP_ERROR(this->get_logger(), "CAN call timed out");
      response->success = false;
    }
    RCLCPP_INFO(this->get_logger(), "Received degrees: %f", request->degrees);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ClawService>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
#include "guppy_msgs/srv/send_claw.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <cstring>
#include <memory>
#include <net/if.h>
#include <unistd.h>

/*
  Recieves a SendClaw request containing a float64 degrees, which is the angle
  the claw is instructed to move. If a valid degrees was sent (0-180 inclusive),
  and the program does not time out, then degrees is can_tx via a SendCan request.
*/ 
class ClawService : public rclcpp::Node
{
public:
  ClawService() : Node("claw_service")
  {
    service_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    client_cbg_ = this->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    // registers the client_ member in ClawService
    // NOTE: uses rmw_qos_profile_services_default as a parameter to set it up normally
    client_ = this->create_client<guppy_msgs::srv::SendCan>(
      "can_tx",
      rmw_qos_profile_services_default,
      client_cbg_);

    // registers the servic in ClawService
    // NOTE: uses rmw_qos_profile_services_default as a parameter to set it up normally
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

  // send() takes in a SendClaw Request (degrees) and Response (bool) and edits the arguments
  // and sends the request to can_tx if its contains a valid degrees
  void send(
    const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request,
    std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response)
  {

    // if degrees is an invalid value (not between 0 and 180), stop the method and set success to false
    if (request->degrees < 0.0 || request->degrees > 180.0) {
      RCLCPP_WARN(this->get_logger(), "Degrees out of range: %f", request->degrees);
      response->success = false;
      return;
    }

    // create and set values for a new can_request
    auto can_request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
    can_request->id = 0x41A;
    can_request->data.resize(sizeof(double));
  
    // degrees is squeezed into the can_request using memcpy
    std::memcpy(
      can_request->data.data(),
      &request->degrees,
      sizeof(double));

    // result will update with a value if it gets recieved in less than 2 seconds
    auto result = client_->async_send_request(can_request);

    // If a response is received for the can_request in under 2 minutes, set responsse to true, otherwise false
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
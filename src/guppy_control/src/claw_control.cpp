/*
  This is a service that takes in a double that is the amount of 
  degrees to move the claw, and sends that double to a client
  that calls SendCan, which translates that into CAN language.
*/

#include "guppy_msgs/srv/send_claw.hpp"
#include "guppy_msgs/srv/send_can.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cstdio>
#include <cstring>
#include <memory>

#include <net/if.h>
#include <unistd.h>

// When a request is made to SendClaw service, this method gets called with the request data and response data
// this method also takes in a parameter for a client to access SendCan
void send(const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request, 
  std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response,
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client,
  std::shared_ptr<rclcpp::Node> node
) {
  
    // create a request
    auto can_request = std::make_shared<guppy_msgs::srv::SendCan::Request>();
    can_request->id = 0x000;


    // this was not working:
    //memcpy(&(can_request->data), &(request->degrees), 8);


    // this maybe is working (?):
    can_request->data.resize(sizeof(double));
    std::memcpy(
        can_request->data.data(),
        &request->degrees,
        sizeof(double)
    );



    // send the request
    auto result = client->async_send_request(can_request);

    // await the result
    rclcpp::spin_until_future_complete(node, result);

    // response
    // ?
}

int main(int argc, char** argv) {
  // Initializes ROS 2 C++ client library:
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("claw_service"); // named claw_tx because I think this transmits data

  // this creates a client so that when a request is made to the
  // SendClaw service, the send() method that is called will take in
  // a pointer to this client and will be able to send data to CAN
  rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client =
    node->create_client<guppy_msgs::srv::SendCan>("can_tx");


  //youll need to edit this to be your srv type
  // guppy_msgs/srv
  auto service = node->create_service<guppy_msgs::srv::SendClaw>(
      "claw_tx", [](
        const std::shared_ptr<guppy_msgs::srv::SendClaw::Request> request,
        std::shared_ptr<guppy_msgs::srv::SendClaw::Response> response,
        rclcpp::Client<guppy_msgs::srv::SendCan>::SharedPtr client,
        std::shared_ptr<rclcpp::Node> node)
          { send(request, response, client, node); }, 10);
      //look at the docs for how to make this simpler

  RCLCPP_INFO(node->get_logger(), "CLAW TX service ready"); // example for how to do logging

  // this line makes the service available
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}



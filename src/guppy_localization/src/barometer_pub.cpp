#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "std_msgs/msg/byte_multi_array.hpp"

using namespace std::chrono_literals;

class BarometerPublisher : public rclcpp::Node {
public:
  BarometerPublisher() : Node("barometer_publisher") {
    this->declare_parameter("can_id", 0x001);
    this->declare_parameter("tf_frame", "barometer");

    char can_topic[30];
    sprintf(can_topic, "/can/id_%x", (unsigned int)this->get_parameter("can_id").as_int());

    publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/barometer", 10);
    subscription_ = this->create_subscription<std_msgs::msg::ByteMultiArray>(
      can_topic, 10,
      std::bind(&BarometerPublisher::callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "setup publisher and subscriber");
  }

  void callback(std_msgs::msg::ByteMultiArray::SharedPtr msg) {
    double depth = 0;
    memcpy(&depth, msg->data.data(), sizeof(double));

    geometry_msgs::msg::PoseWithCovarianceStamped pose_out;
    pose_out.header.frame_id = this->get_parameter("tf_frame").as_string();
    pose_out.header.stamp = this->get_clock()->now();
    pose_out.pose.pose.position.z = depth;

    // this is chatgpt... need to find actual values
    pose_out.pose.covariance = {
      -1, 0, 0, 0, 0, 0,
      0, -1, 0, 0, 0, 0,
      0, 0, 0.5, 0, 0, 0,
      0, 0, 0, -1, 0, 0,
      0, 0, 0, 0, -1, 0,
      0, 0, 0, 0, 0, -1
    };

    publisher_->publish(pose_out);

    RCLCPP_DEBUG(this->get_logger(), "sent pose with altitude %f", depth);
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::ByteMultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BarometerPublisher>());
  rclcpp::shutdown();
  return 0;
}
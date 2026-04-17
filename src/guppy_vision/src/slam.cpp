#include <map>
#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "guppy_msgs/msg/transform_list.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

class SlamNode : public rclcpp::Node {
public:
  SlamNode() : Node("slam") {
    output_pub_ = this->create_subscription<guppy_msgs::msg::TransformList>(
      "/cam/test/transforms", 10,
      std::bind(&SlamNode::callback, this, std::placeholders::_1)
    );

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

  }

  ~SlamNode() {
    // ...
  }

  void callback(guppy_msgs::msg::TransformList::UniquePtr msg) {
    for (geometry_msgs::msg::TransformStamped transform: msg->transforms) {
      std::string name = transform.child_frame_id;


      if (name == "tag_10") {
        RCLCPP_INFO(this->get_logger(), "updated map transform...");
        geometry_msgs::msg::TransformStamped map_transform;
        map_transform.child_frame_id = transform.child_frame_id;
        map_transform.header.frame_id = "map";

        tf2::Transform tfTransform;
        tf2::fromMsg(transform.transform, tfTransform);

        map_transform.transform = tf2::toMsg(tfTransform);
        tf_static_broadcaster_->sendTransform(map_transform);

        // has_map_transform = true;
      }
      tf_broadcaster_->sendTransform(transform);
    }
  }

private:
    rclcpp::Subscription<guppy_msgs::msg::TransformList>::SharedPtr output_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    bool has_map_transform = false;
    std::map<std::string, bool> already_seen;
    std::map<std::string, tf2::Transform> map_to_obj;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<SlamNode>());

  rclcpp::shutdown();

  return 0;
}
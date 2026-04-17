#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <Eigen/Geometry>

class CombineSensors : public rclcpp::Node {
public:
  CombineSensors() : Node("combine_sensors") {
    imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>("/vectornav/imu",10,std::bind(&CombineSensors::imu_callback, this, std::placeholders::_1));
    dvl_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/waterlinked_dvl_driver/odom",10,std::bind(&CombineSensors::dvl_callback, this, std::placeholders::_1));
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odometry/filtered", 10);
  }

  ~CombineSensors() {
    // ...
  }

  void dvl_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    Eigen::Vector3d axis_vector(0.0, 1.0, 0.0);
    Eigen::AngleAxisd angle_axis(M_PI, axis_vector);
    Eigen::Quaterniond quat(angle_axis);

    Eigen::Vector3d position(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d twist(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);

    position = quat * position;
    twist = quat * twist;

    odom.pose.pose.position.x = position[0];
    odom.pose.pose.position.y = position[1];
    odom.pose.pose.position.z = position[2];
    odom.twist.twist.linear.x = -1 * twist[0];

    double rx = 0.2;
    double ry = 0.0;
    double rz = -0.125;
    double wx = odom.twist.twist.angular.x;
    double wy = odom.twist.twist.angular.y;
    double wz = odom.twist.twist.angular.z;

    odom.twist.twist.linear.x = twist[0] - (wy*rz - wz*ry);
    odom.twist.twist.linear.y = twist[1] - (wz*rx - wx*rz);
    odom.twist.twist.linear.z = twist[2] - (wx*ry - wy*rx);

    odom.twist.twist.linear.x *= -1;
    odom_pub_->publish(odom);
  }

  void imu_callback(sensor_msgs::msg::Imu::SharedPtr msg) {
    Eigen::Vector3d axis_vector(0.0, 0.0, 1.0);
    Eigen::AngleAxisd angle_axis(M_PI * -0.5, axis_vector);
    Eigen::Quaterniond quat(angle_axis);

    Eigen::Quaternion orientation(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
    Eigen::Vector3d twist(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

    orientation = orientation * quat;
    twist = quat * twist;

    odom.pose.pose.orientation.w = orientation.w();
    odom.pose.pose.orientation.x = orientation.x();
    odom.pose.pose.orientation.y = orientation.y();
    odom.pose.pose.orientation.z = orientation.z();
    odom.twist.twist.angular.x = twist[0];
    odom.twist.twist.angular.y = twist[1];
    odom.twist.twist.angular.z = twist[2];
    odom_pub_->publish(odom);
  }

private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr dvl_subscription_;

  nav_msgs::msg::Odometry odom;

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<CombineSensors>());

  rclcpp::shutdown();

  return 0;
}
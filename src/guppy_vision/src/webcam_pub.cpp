#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class WebcamPublisher : public rclcpp::Node {
public:
  WebcamPublisher() : Node("webcam_pub") {
    camera.open(1);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/cam/test/raw", 10);
    info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cam/test/info", 10);
    timer_ = this->create_wall_timer(20ms, std::bind(&WebcamPublisher::timer_callback, this));
  }

  ~WebcamPublisher() {
    // ...
  }

private:
    void timer_callback() {
        camera.grab();
        cv::Mat image;
        camera.retrieve(image);

        msg_ = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_pub_->publish(*msg_.get());

        sensor_msgs::msg::CameraInfo info;
        info.height = image.rows;
        info.width = image.cols;
        info.distortion_model = "dummy";
        info.d = {4.96594686e+02, -5.59325329e-01, 6.24601548e+00, -2.55300791e+00, -3.58715498e-04};
        info.k = {
            8.49762156e+03, 0.00000000e+00, 3.01301941e+02,
            0.00000000e+00, 6.30069077e+03, 2.39384146e+02,
            0.00000000e+00, 0.00000000e+00, 1.00000000e+00
        };

        info_pub_->publish(info);
    }

    cv::VideoCapture camera;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr info_pub_;
    sensor_msgs::msg::Image::SharedPtr msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<WebcamPublisher>());

  rclcpp::shutdown();

  return 0;
}
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
    camera.open(0);

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

        rclcpp::Time now = this->get_clock()->now();
        std_msgs::msg::Header header;
        header.frame_id = "test";
        header.stamp = now;

        msg_ = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();
        image_pub_->publish(*msg_.get());

        sensor_msgs::msg::CameraInfo info;
        info.height = image.rows;
        info.width = image.cols;
        info.distortion_model = "opencv(?)";
        info.d = {0.11278175177017583, -0.2634338012261266, -0.004302324657789871, 0.0007975991434697732, 0.16261327039070175};
        info.k = {
          556.6559171023735,    0.0,                329.6728647483841,
          0.0,                  556.1111124909132,  240.66490808614884,
          0.0,                  0.0,                1.0
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
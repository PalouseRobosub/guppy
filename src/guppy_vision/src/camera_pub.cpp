#include <memory>
#include <atomic>
#include <Spinnaker.h>
#include <thread>
// #include <SpinGenApi/SpinnakerGenApi.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/fill_image.hpp"

class CameraPublisher : public rclcpp::Node {
public:
  CameraPublisher() : Node("camera_publisher") {
    this->running_.store(true);

    this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cam0", 1);

    this->system  = Spinnaker::System::GetInstance();
    this->camList = system->GetCameras();

    this->ackThread = std::thread(&CameraPublisher::acquireFootage, this);

  }

  ~CameraPublisher() {
    this->running_.store(false);
    this->camList.Clear();
    this->system->ReleaseInstance();
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camList;

  std::thread ackThread;
  std::atomic<bool> running_{false};

  void acquireFootage() {
    Spinnaker::CameraPtr pCam = this->camList[0];
    pCam->Init();
    pCam->BeginAcquisition();
    Spinnaker::ImageProcessor processor;
    while (this->running_) {
      Spinnaker::ImagePtr pImg = pCam->GetNextImage(1000);
      Spinnaker::ImagePtr pProcessed = processor.Convert(pImg, Spinnaker::PixelFormat_RGB8);
      sensor_msgs::msg::Image imageMsg;
      sensor_msgs::fillImage(
        imageMsg,
        "rgb8",
        pProcessed->GetHeight(),
        pProcessed->GetWidth(),
        pProcessed->GetStride(),
        pProcessed->GetData()
        );

      std::cout << "publish" << std::endl;

      this->publisher_->publish(imageMsg);
    }
  }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraPublisher>());
    rclcpp::shutdown();
    return 0;
}
#include <CameraDefs.h>
#include <Spinnaker.h>
#include <atomic>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
// #include <SpinGenApi/SpinnakerGenApi.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/fill_image.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraPublisher : public rclcpp::Node {
 public:
  CameraPublisher() : Node("camera_publisher") {
    this->running_.store(true);

    // this->publisher_ = this->create_publisher<sensor_msgs::msg::Image>("cam0", 1);

    this->system = Spinnaker::System::GetInstance();
    this->camList = system->GetCameras();

    for (unsigned int i = 0; i < camList.GetSize(); ++i) {
      std::string name = "cam";
      name += i;
      std::cout << name << std::endl;
      this->publishers_.push_back(this->create_publisher<sensor_msgs::msg::Image>(name, 1));
      this->threads_.emplace_back(&CameraPublisher::acquireFootage, this, i);
    }
  }

  ~CameraPublisher() {
    this->running_.store(false);

    for (auto& t : threads_) {
        if (t.joinable()) {
            t.join();
        }
    }

    this->camList.Clear();
    this->system->ReleaseInstance();
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;

  Spinnaker::SystemPtr system;
  Spinnaker::CameraList camList;

  std::thread ackThread;
  std::atomic<bool> running_{false};

  std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> publishers_;
  std::vector<std::thread> threads_;

  void acquireFootage(int camIndex) {
    Spinnaker::CameraPtr pCam = this->camList[camIndex];
    pCam->Init();
    pCam->BeginAcquisition();
    Spinnaker::ImageProcessor processor;
    while (this->running_) {
      Spinnaker::ImagePtr pImg = pCam->GetNextImage(1000);
      Spinnaker::ImagePtr pProcessed = processor.Convert(pImg, Spinnaker::PixelFormat_Mono8);
      auto msg = std::make_unique<sensor_msgs::msg::Image>();
      sensor_msgs::fillImage(*msg, "mono8", pProcessed->GetHeight(), pProcessed->GetWidth(), pProcessed->GetStride(), pProcessed->GetData());
      pImg->Release();

      this->publishers_[camIndex]->publish(std::move(msg));
    }
  }
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraPublisher>());
  rclcpp::shutdown();
  return 0;
}

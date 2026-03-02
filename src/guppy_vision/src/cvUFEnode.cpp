#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <map>
#include <string>

struct CameraState{
    bool calibrated = false;
    cv::Mat K;
    cv::Mat D;
    cv:: Mat map1, map2;
    cv::Size img_size;

};

//COMPUTER VISION UN-DISTORT
class cvud : public rclcpp::Node{
    public:

    cvud() : Node("Camera_Undistort_Node"){
        const std::vector<std::string> cameras = {"raw_1","raw_2","raw_3","raw_4"};
        cv::Size img_size(640, 480);
        cv::Mat K = (cv::Mat_<double>(3,3) <<
            320.0,   0.0, 320.0,
              0.0, 320.0, 240.0,
              0.0,   0.0,   1.0);

        cv::Mat D = (cv::Mat_<double>(4,1) << -0.3, 0.1, 0.0, 0.0);

        for (const auto & cam : cameras){
            const std::string base      = "/guppy/" + cam;
            const std::string in_img    = base;
            const std::string out_topic = base + "/image_undistorted";

            CameraState state;
            cv::Mat new_K;
            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
                K, D, img_size,
                cv::Mat::eye(3,3,CV_64F),
                new_K,
                0.0,
                img_size);

            cv::fisheye::initUndistortRectifyMap(
                K, D,
                cv::Mat::eye(3,3,CV_64F),
                new_K,
                img_size,
                CV_16SC2,
                state.map1,
                state.map2);

            state.calibrated = true;
            states_[cam] = state;

            pub_[cam] = this->create_publisher<sensor_msgs::msg::Image>(out_topic, 10);

            sub_img_[cam] = this->create_subscription<sensor_msgs::msg::Image>(
                in_img, 10,
                [this, cam](sensor_msgs::msg::Image::ConstSharedPtr msg){
                    image_cb(cam, msg);
                });

            RCLCPP_INFO(this->get_logger(),
                "Camera '%s': listening on '%s', publishing to '%s'",
                cam.c_str(), in_img.c_str(), out_topic.c_str());
        }
    }
    private:

    // void camera_info_cb(const std::string & cam, sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
    //     auto & state = states_[cam];
    //     if(state.calibrated){
    //         return;
    //     }
    //     state.img_size = cv::Size(msg->width, msg->height);

    //     state.K = (cv::Mat_<double>(3,3) << msg->k[0], msg->k[1], msg->k[2], msg->k[3], msg->k[4], msg->k[5], msg->k[6], msg->k[7], msg->k[8]);

    //     if(msg->d.size() >= 4){
    //         state.D = (cv::Mat_<double>(4,1) << msg->d[0], msg->d[1], msg->d[2], msg->d[3]);
    //     }else{
    //         state.D = cv::Mat::zeros(4,1,CV_64F);
    //         RCLCPP_WARN(this->get_logger(), "[%s] camera_info has fewer than 4 distortion coeffs"
    //         "assuming zero distortion.", cam.c_str());
    //     }
        
    //     cv::Mat new_K;
    //     cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
    //         state.K,
    //         state.D,
    //         state.img_size,
    //         cv::Mat::eye(3,3,CV_64F),
    //         new_K,        
    //         0.0,
    //         state.img_size
    //     );
    //     cv::fisheye::initUndistortRectifyMap(
    //         state.K,
    //         state.D,
    //         cv::Mat::eye(3,3,CV_64F),
    //         new_K,
    //         state.img_size,
    //         CV_16SC2,
    //         state.map1,
    //         state.map2
    //     );
    //     state.calibrated = true;
    //     RCLCPP_INFO(this->get_logger(),"[%s] Undistortion maps ready (%dx%d).",cam.c_str(), msg->width, msg->height);
    // }

    void image_cb(const std::string & cam, sensor_msgs::msg::Image::ConstSharedPtr msg){
        auto & state = states_[cam];
        cv_bridge::CvImageConstPtr cv_ptr;
        try{
            cv_ptr = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8);
        }catch (const cv_bridge::Exception & e){
            RCLCPP_ERROR(this->get_logger(),"[%s] cv_bridge exception: %s", cam.c_str(), e.what());
            return;
        }

        cv::Mat output;

        if(state.calibrated){
            cv::remap(cv_ptr->image,output,state.map1,state.map2,cv::INTER_LINEAR,cv::BORDER_CONSTANT);
        }else{
            output = cv_ptr->image.clone();
            RCLCPP_WARN_THROTTLE(this->get_logger(),*this->get_clock(),5000,"[%s] Waiting for camera_info to build undistortion maps",cam.c_str());
        }
        auto out_msg = cv_bridge::CvImage(
            msg->header,
            sensor_msgs::image_encodings::BGR8,
            output).toImageMsg();
        
        pub_[cam]->publish(*out_msg);
    }
    std::map<std::string, CameraState> states_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr>   pub_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> sub_info_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr>      sub_img_;
};

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<cvud>());
    rclcpp::shutdown();
    return 0;
}
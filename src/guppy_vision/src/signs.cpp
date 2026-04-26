#include <memory>
#include <rclcpp/node.hpp>
#include "rclcpp/rclcpp.hpp"
#include <image_transport/publisher.hpp>
#include <image_transport/subscriber.hpp>
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

using std::vector;
using cv::Mat;
using cv::Point;
using cv::Point2f;
using cv::RotatedRect;
using cv::Ptr;
using cv::CLAHE;
using cv::Size;

class SignDetector : public rclcpp::Node {
public:
    SignDetector() : Node("detect_signs") {
        auto qos = rclcpp::BestAvailableQoS();
        this->image_sub_ = image_transport::create_subscription(this, "/cam/test", std::bind(&SignDetector::cb, this, std::placeholders::_1), "raw", qos.get_rmw_qos_profile());
        this->image_pub_ = image_transport::create_publisher(this, "/cam/signs", qos.get_rmw_qos_profile());
        this->clahe = cv::createCLAHE(2.0, {8, 8});
    }
    
    
private:
    Mat in_;
    Mat prevGray_;
    Ptr<CLAHE> clahe;
    image_transport::Publisher image_pub_;
    image_transport::Subscriber image_sub_;
    vector<vector<Point>> squares_;
        
    double angle(Point pt1, Point pt2, Point pt0)
    {
        double dx1 = pt1.x - pt0.x;
        double dy1 = pt1.y - pt0.y;
        double dx2 = pt2.x - pt0.x;
        double dy2 = pt2.y - pt0.y;
        return (dx1*dx2 + dy1*dy2) / sqrt((dx1*dx1 + dy1*dy1) * (dx2*dx2 + dy2*dy2) + 1e-10);
    }
    
    void findWhiteSquares(const Mat& image) {
        squares_.clear();
        
        Mat pyr, timg;
        pyrDown(image, pyr, Size(image.cols/2, image.rows/2));
        pyrUp(pyr, timg, image.size());
        
        Mat gray;
        cvtColor(timg, gray, cv::COLOR_BGR2GRAY);
            
        clahe->apply(gray, gray);
        
        if (!this->prevGray_.empty()) {
                addWeighted(gray, 0.6, this->prevGray_, 0.4, 0, gray);
            }
        gray.copyTo(this->prevGray_);
        
        GaussianBlur(gray, gray, Size(5, 5), 0);
        
        Mat canny;
        Canny(gray, canny, 40, 90, 3);
        dilate(canny, canny, Mat());
        
        vector<vector<Point>> contours;
        
        findContours(canny, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        // imshow("live", canny);
        
        vector<Point> approx;
        // test each contour
        for( size_t i = 0; i < contours.size(); i++ )
        {
            approxPolyDP(contours[i], approx, arcLength(contours[i], true)*0.02, true);
            
            if (approx.size() < 4 || approx.size() > 6) continue;
            if (fabs(contourArea(approx)) < 500) continue;
            if (!isContourConvex(approx)) continue;
            
            double maxCosine = 0;
                
            for( int j = 2; j < 5; j++ )
            {
                // find the maximum cosine of the angle between joint edges
                double cosine = fabs(angle(approx[j%4], approx[j-2], approx[j-1]));
                maxCosine = MAX(maxCosine, cosine);
            }
            
            if (maxCosine < 0.7) {
                RotatedRect rr = minAreaRect(approx);
                Point2f pts[4];
                rr.points(pts);
                this->squares_.push_back(vector<Point>(pts, pts + 4));
            }
        } 
    }
    
    void cb(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        cv_bridge::CvImagePtr cv_ptr;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        this->in_ = cv_ptr->image;
        
        this->findWhiteSquares(this->in_);
        
        polylines(this->in_, this->squares_, true, cv::Scalar(0, 255, 0), 3, cv::LINE_AA);
        
        cv_ptr->image = this->in_;
        rclcpp::Time now = this->get_clock()->now();
        cv_ptr->header.set__stamp(now);
        cv_ptr->header.set__frame_id("signs");
            
        this->image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<SignDetector>();
    
    rclcpp::spin(node);
    
    rclcpp::shutdown();
}
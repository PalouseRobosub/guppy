// scripts/lane_navigator_cpp.cpp

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ros_gz_interfaces/msg/altimeter.hpp>
//THESE DEPENDENCIES WILL PROBABLY CAUSE AN ERROR, IGNORE THEM AS COLCON WILL COMPILE THEM ANYWAYS.
//THIS IS AN INTELLISENSE PROBLEM IDK HOW TO FIX IT
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <cmath>
#include <algorithm>
#include <vector>
#include <string>

using namespace std;
using namespace cv;

using std::placeholders::_1;
using namespace std::chrono_literals;

static inline double clampd(double v, double lo, double hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline int clampi(int v, int lo, int hi)
{
  return std::max(lo, std::min(hi, v));
}

static inline void quat_to_euler_xyz(double qx, double qy, double qz, double qw,
                                     double &roll, double &pitch, double &yaw)
{
  const double sinr_cosp = 2.0 * (qw * qx + qy * qz);
  const double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
  roll = std::atan2(sinr_cosp, cosr_cosp);

  const double sinp = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sinp) >= 1.0)
    pitch = std::copysign(M_PI / 2.0, sinp);
  else
    pitch = std::asin(sinp);

  const double siny_cosp = 2.0 * (qw * qz + qx * qy);
  const double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
  yaw = std::atan2(siny_cosp, cosy_cosp);
}

class LaneNavigator : public rclcpp::Node
{
public:
  LaneNavigator() : Node("lane_navigator_cpp")
  {
    this->declare_parameter<bool>("enable_debug", true);
    this->declare_parameter<bool>("show_cv_windows", false);   // imshow
    this->declare_parameter<bool>("publish_debug_images", true);

    this->declare_parameter<int>("debug_show_every_n_frames", 2);

    this->declare_parameter<double>("target_depth", 1.0);
    this->declare_parameter<bool>("depth_increases_down", true);
    this->declare_parameter<double>("kp_depth", 4.0);
    this->declare_parameter<double>("ki_depth", 0.1);
    this->declare_parameter<double>("kd_depth", 0.5);
    this->declare_parameter<double>("max_heave_cmd", 2.0);

    this->declare_parameter<double>("kp_roll", 3.0);
    this->declare_parameter<double>("kd_roll", 0.8);
    this->declare_parameter<double>("kp_pitch", 3.0);
    this->declare_parameter<double>("kd_pitch", 0.8);
    this->declare_parameter<double>("max_att_cmd", 2.0);

    this->declare_parameter<double>("kp_yaw_px", 0.006);
    this->declare_parameter<double>("kd_yaw", 0.05);
    this->declare_parameter<double>("k_lane_theta", 0.6);
    this->declare_parameter<double>("max_yaw", 1.2);
    this->declare_parameter<double>("search_yaw", 0.35);
    this->declare_parameter<double>("lane_lookahead_frac", 0.65);
    this->declare_parameter<double>("lane_found_timeout", 0.5);

    this->declare_parameter<double>("forward_speed", 0.35);
    this->declare_parameter<double>("forward_speed_search", 0.05);

    this->declare_parameter<std::vector<long>>("hsv_red1_low",  {0,   100, 100});
    this->declare_parameter<std::vector<long>>("hsv_red1_high", {10,  255, 255});
    this->declare_parameter<std::vector<long>>("hsv_red2_low",  {170, 100, 100});
    this->declare_parameter<std::vector<long>>("hsv_red2_high", {180, 255, 255});
    this->declare_parameter<std::vector<long>>("hsv_white_low",  {0,   0,   55});
    this->declare_parameter<std::vector<long>>("hsv_white_high", {179, 45, 255});

    this->declare_parameter<int>("min_contour_area", 300);
    this->declare_parameter<double>("min_aspect", 2.0);
    this->declare_parameter<int>("blur_ksize", 5);
    this->declare_parameter<int>("morph_ksize", 5);
    this->declare_parameter<int>("max_poles_each_color", 6);

    debug_ = this->get_parameter("enable_debug").as_bool();
    show_cv_ = this->get_parameter("show_cv_windows").as_bool();
    publish_debug_ = this->get_parameter("publish_debug_images").as_bool();

    debug_every_n_ = std::max<int>(
      1, static_cast<int>(this->get_parameter("debug_show_every_n_frames").as_int())
    );

    target_depth_ = this->get_parameter("target_depth").as_double();
    depth_increases_down_ = this->get_parameter("depth_increases_down").as_bool();
    kp_depth_ = this->get_parameter("kp_depth").as_double();
    ki_depth_ = this->get_parameter("ki_depth").as_double();
    kd_depth_ = this->get_parameter("kd_depth").as_double();
    max_heave_cmd_ = this->get_parameter("max_heave_cmd").as_double();

    kp_roll_ = this->get_parameter("kp_roll").as_double();
    kd_roll_ = this->get_parameter("kd_roll").as_double();
    kp_pitch_ = this->get_parameter("kp_pitch").as_double();
    kd_pitch_ = this->get_parameter("kd_pitch").as_double();
    max_att_cmd_ = this->get_parameter("max_att_cmd").as_double();

    kp_yaw_px_ = this->get_parameter("kp_yaw_px").as_double();
    kd_yaw_ = this->get_parameter("kd_yaw").as_double();
    k_lane_theta_ = this->get_parameter("k_lane_theta").as_double();
    max_yaw_ = this->get_parameter("max_yaw").as_double();
    search_yaw_ = this->get_parameter("search_yaw").as_double();
    lane_lookahead_frac_ = this->get_parameter("lane_lookahead_frac").as_double();
    lane_found_timeout_ = this->get_parameter("lane_found_timeout").as_double();

    forward_speed_ = this->get_parameter("forward_speed").as_double();
    forward_speed_search_ = this->get_parameter("forward_speed_search").as_double();

    min_area_ = this->get_parameter("min_contour_area").as_int();
    min_aspect_ = this->get_parameter("min_aspect").as_double();
    blur_ksize_ = this->get_parameter("blur_ksize").as_int();
    morph_ksize_ = this->get_parameter("morph_ksize").as_int();
    max_poles_each_color_ = this->get_parameter("max_poles_each_color").as_int();

    red1_low_  = vec_to_scalar(this->get_parameter("hsv_red1_low").as_integer_array());
    red1_high_ = vec_to_scalar(this->get_parameter("hsv_red1_high").as_integer_array());
    red2_low_  = vec_to_scalar(this->get_parameter("hsv_red2_low").as_integer_array());
    red2_high_ = vec_to_scalar(this->get_parameter("hsv_red2_high").as_integer_array());
    white_low_  = vec_to_scalar(this->get_parameter("hsv_white_low").as_integer_array());
    white_high_ = vec_to_scalar(this->get_parameter("hsv_white_high").as_integer_array());

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    auto sensor_qos = rclcpp::SensorDataQoS();  

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/cube/image_raw", sensor_qos, std::bind(&LaneNavigator::image_cb, this, _1));

    alt_sub_ = this->create_subscription<ros_gz_interfaces::msg::Altimeter>(
      "/altimeter", sensor_qos, std::bind(&LaneNavigator::alt_cb, this, _1));

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu", sensor_qos, std::bind(&LaneNavigator::imu_cb, this, _1));

    if (publish_debug_) {
      dbg_rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_debug/rgb", 1);
      dbg_red_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_debug/red_mask", 1);
      dbg_white_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_debug/white_mask", 1);
      dbg_overlay_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/lane_debug/overlay", 1);
    }

    last_lane_seen_time_ = this->now();
    prev_time_ = this->now();
    timer_ = this->create_wall_timer(10ms, std::bind(&LaneNavigator::control_loop, this));

    if (show_cv_) {
      cv::namedWindow("rgb", cv::WINDOW_NORMAL);
      cv::namedWindow("red_mask", cv::WINDOW_NORMAL);
      cv::namedWindow("white_mask", cv::WINDOW_NORMAL);
      cv::namedWindow("overlay", cv::WINDOW_NORMAL);
    }

    RCLCPP_INFO(this->get_logger(),
                "lane_navigator_cpp running (100 Hz). publish_debug_images=%s show_cv_windows=%s debug_every_n=%d",
                publish_debug_ ? "true" : "false",
                show_cv_ ? "true" : "false",
                debug_every_n_);
  }

private:
  static cv::Scalar vec_to_scalar(const std::vector<long> &v)
  {
    if (v.size() != 3) return cv::Scalar(0, 0, 0);
    return cv::Scalar(static_cast<double>(v[0]),
                      static_cast<double>(v[1]),
                      static_cast<double>(v[2]));
  }

  struct Pole
  {
    int cx{0}, cy{0};
    double area{0.0};
    cv::Rect bbox;
  };

  void image_cb(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
      image_ = cv_ptr->image.clone();
      have_image_ = true;
    } catch (const std::exception &e) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "cv_bridge conversion failed: %s", e.what());
    }
  }

  void alt_cb(const ros_gz_interfaces::msg::Altimeter::SharedPtr msg)
  {
    double z = static_cast<double>(msg->vertical_position);
    depth_ = depth_increases_down_ ? z : -z;
    have_depth_ = true;
  }

  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    quat_to_euler_xyz(msg->orientation.x, msg->orientation.y,
                      msg->orientation.z, msg->orientation.w,
                      roll_, pitch_, yaw_);
    roll_rate_ = msg->angular_velocity.x;
    pitch_rate_ = msg->angular_velocity.y;
    yaw_rate_ = msg->angular_velocity.z;
    have_imu_ = true;
  }

  std::vector<Pole> pole_centroids(const cv::Mat &mask)
  {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::vector<Pole> poles;
    poles.reserve(contours.size());

    for (const auto &c : contours) {
      double area = cv::contourArea(c);
      if (area < static_cast<double>(min_area_)) continue;

      cv::Rect r = cv::boundingRect(c);
      if (r.width <= 0) continue;
      double aspect = static_cast<double>(r.height) / static_cast<double>(r.width);
      if (aspect < min_aspect_) continue;

      cv::Moments M = cv::moments(c);
      if (std::abs(M.m00) < 1e-6) continue;

      Pole p;
      p.cx = static_cast<int>(M.m10 / M.m00);
      p.cy = static_cast<int>(M.m01 / M.m00);
      p.area = area;
      p.bbox = r;
      poles.push_back(p);
    }

    std::sort(poles.begin(), poles.end(), [](const Pole &a, const Pole &b) {
      if (a.cy != b.cy) return a.cy > b.cy;
      return a.area > b.area;
    });

    if (static_cast<int>(poles.size()) > max_poles_each_color_) {
      poles.resize(max_poles_each_color_);
    }
    return poles;
  }

  bool pick_best_pair(const std::vector<Pole> &reds,
                      const std::vector<Pole> &whites,
                      Pole &best_r,
                      Pole &best_w)
  {
    if (reds.empty() || whites.empty()) return false;

    const int NR = std::min<int>(3, static_cast<int>(reds.size()));
    const int NW = std::min<int>(3, static_cast<int>(whites.size()));

    bool found = false;
    double best_score = 1e18;

    for (int i = 0; i < NR; ++i) {
      for (int j = 0; j < NW; ++j) {
        const auto &r = reds[i];
        const auto &w = whites[j];

        if (r.cx >= w.cx) continue;

        const double dy = std::abs(r.cy - w.cy);
        const double area_bonus = 0.0005 * (r.area + w.area);
        const double score = dy - area_bonus;

        if (score < best_score) {
          best_score = score;
          best_r = r;
          best_w = w;
          found = true;
        }
      }
    }
    return found;
  }

  bool detect_lane(const cv::Mat &bgr, double &x_center, double &lane_theta,
                   cv::Mat &red_mask_out, cv::Mat &white_mask_out, cv::Mat &overlay_out)
  {
    cv::Mat img = bgr;

    if (blur_ksize_ > 1) {
      int k = (blur_ksize_ % 2 == 1) ? blur_ksize_ : (blur_ksize_ + 1);
      cv::GaussianBlur(img, img, cv::Size(k, k), 0.0);
    }

    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    cv::Mat red1, red2, red_mask, white_mask;
    cv::inRange(hsv, red1_low_, red1_high_, red1);
    cv::inRange(hsv, red2_low_, red2_high_, red2);
    red_mask = red1 | red2;
    cv::inRange(hsv, white_low_, white_high_, white_mask);

    int mk = (morph_ksize_ % 2 == 1) ? morph_ksize_ : (morph_ksize_ + 1);
    mk = std::max(1, mk);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(mk, mk));

    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

    cv::morphologyEx(white_mask, white_mask, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 1);
    cv::morphologyEx(white_mask, white_mask, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 2);

    overlay_out = bgr.clone();
    overlay_out.setTo(cv::Scalar(0, 0, 255), red_mask);
    overlay_out.setTo(cv::Scalar(255, 255, 255), white_mask);

    auto red_poles = pole_centroids(red_mask);
    auto white_poles = pole_centroids(white_mask);

    for (const auto &p : red_poles) {
      cv::rectangle(overlay_out, p.bbox, cv::Scalar(0, 0, 255), 2);
      cv::circle(overlay_out, cv::Point(p.cx, p.cy), 3, cv::Scalar(0, 0, 255), -1);
    }
    for (const auto &p : white_poles) {
      cv::rectangle(overlay_out, p.bbox, cv::Scalar(200, 200, 200), 2);
      cv::circle(overlay_out, cv::Point(p.cx, p.cy), 3, cv::Scalar(200, 200, 200), -1);
    }

    red_mask_out = red_mask;
    white_mask_out = white_mask;

    Pole best_r{}, best_w{};
    const bool have_pair = pick_best_pair(red_poles, white_poles, best_r, best_w);
    if (have_pair)
    {
      const cv::Point mid((best_r.cx + best_w.cx) / 2, (best_r.cy + best_w.cy) / 2);

      cv::circle(overlay_out, mid, 8, cv::Scalar(0, 255, 255), -1);

      cv::line(overlay_out, cv::Point(best_r.cx, best_r.cy), cv::Point(best_w.cx, best_w.cy),
               cv::Scalar(0, 255, 255), 2);

      x_center = static_cast<double>(mid.x);
      lane_theta = 0.0; 
    }
    else
    {
      return false; 
    }

    if (red_poles.size() < 2 || white_poles.size() < 2) {
      return true; 
    }

    const int H = bgr.rows;
    const int W = bgr.cols;
    const int y_look = clampi(static_cast<int>(lane_lookahead_frac_ * H), 0, H - 1);

    auto fit_line = [](const std::vector<Pole> &poles, double &a, double &b) {
      double Syy = 0, Sy = 0, S1 = 0, Sxy = 0, Sx = 0;
      for (const auto &p : poles) {
        const double y = static_cast<double>(p.cy);
        const double x = static_cast<double>(p.cx);
        Syy += y * y;
        Sy  += y;
        S1  += 1.0;
        Sxy += x * y;
        Sx  += x;
      }
      const double det = Syy * S1 - Sy * Sy;
      if (std::abs(det) < 1e-9) { a = 0.0; b = Sx / std::max(1.0, S1); return; }
      a = (Sxy * S1 - Sx * Sy) / det;
      b = (Syy * Sx - Sy * Sxy) / det;
    };

    double ar, br, aw, bw;
    fit_line(red_poles, ar, br);
    fit_line(white_poles, aw, bw);

    const double x_left  = ar * y_look + br;
    const double x_right = aw * y_look + bw;

    if (x_left >= x_right) {
      return true;
    }

    x_center = clampd(0.5 * (x_left + x_right), 0.0, static_cast<double>(W - 1));

    const double a_center = 0.5 * (ar + aw);
    lane_theta = std::atan(a_center);

    auto draw_xy = [&](double a, double b, const cv::Scalar &color) {
      int y0 = 0;
      int y1 = H - 1;
      int x0 = clampi(static_cast<int>(a * y0 + b), -2000, 2000);
      int x1 = clampi(static_cast<int>(a * y1 + b), -2000, 2000);
      cv::line(overlay_out, cv::Point(x0, y0), cv::Point(x1, y1), color, 2);
    };

    draw_xy(ar, br, cv::Scalar(0, 0, 255));
    draw_xy(aw, bw, cv::Scalar(220, 220, 220));

    cv::circle(overlay_out, cv::Point(static_cast<int>(x_center), y_look), 6, cv::Scalar(0, 255, 0), -1);
    cv::line(overlay_out, cv::Point(W / 2, 0), cv::Point(W / 2, H - 1), cv::Scalar(0, 255, 255), 1);

    return true;
  }

  void publish_dbg_images(const cv::Mat &rgb, const cv::Mat &red_mask, const cv::Mat &white_mask, const cv::Mat &overlay)
  {
    if (!publish_debug_) return;
    if (!dbg_rgb_pub_ || !dbg_red_pub_ || !dbg_white_pub_ || !dbg_overlay_pub_) return;

    auto stamp = this->now();

    auto to_msg_bgr = [&](const cv::Mat &bgr_img) {
      cv_bridge::CvImage out;
      out.header.stamp = stamp;
      out.header.frame_id = "camera_link";
      out.encoding = "bgr8";
      out.image = bgr_img;
      return *out.toImageMsg();
    };

    auto to_msg_mono = [&](const cv::Mat &mono_img) {
      cv_bridge::CvImage out;
      out.header.stamp = stamp;
      out.header.frame_id = "camera_link";
      out.encoding = "mono8";
      out.image = mono_img;
      return *out.toImageMsg();
    };

    dbg_rgb_pub_->publish(to_msg_bgr(rgb));

    cv::Mat r = red_mask, w = white_mask;
    if (r.type() != CV_8UC1) cv::cvtColor(r, r, cv::COLOR_BGR2GRAY);
    if (w.type() != CV_8UC1) cv::cvtColor(w, w, cv::COLOR_BGR2GRAY);

    dbg_red_pub_->publish(to_msg_mono(r));
    dbg_white_pub_->publish(to_msg_mono(w));
    dbg_overlay_pub_->publish(to_msg_bgr(overlay));
  }

  void kf_reset(double x0)
{
  kf_inited_ = true;
  kf_x_ = x0;
  kf_v_ = 0.0;

  P00_ = 500.0; P01_ = 0.0;
  P10_ = 0.0;   P11_ = 500.0;
}

void kf_predict(double dt)
{
  // State transition: [x] = [1 dt][x]
  //                   [v]   [0  1][v]
  kf_x_ = kf_x_ + dt * kf_v_;

  // P = A P A^T + Q
  // A = [[1, dt],[0,1]]
  double A00 = 1.0, A01 = dt;
  double A10 = 0.0, A11 = 1.0;

  double P00 = P00_, P01 = P01_, P10 = P10_, P11 = P11_;

  double AP00 = A00*P00 + A01*P10;
  double AP01 = A00*P01 + A01*P11;
  double AP10 = A10*P00 + A11*P10;
  double AP11 = A10*P01 + A11*P11;

  double newP00 = AP00*A00 + AP01*A01;
  double newP01 = AP00*A10 + AP01*A11;
  double newP10 = AP10*A00 + AP11*A01;
  double newP11 = AP10*A10 + AP11*A11;

  // Add process noise (simple diagonal Q)
  newP00 += q_pos_ * dt * dt;
  newP11 += q_vel_ * dt;

  P00_ = newP00; P01_ = newP01;
  P10_ = newP10; P11_ = newP11;
}

void kf_update(double z_meas)
{
  // Measurement: z = H x, H = [1 0]
  // Innovation: y = z - x
  double y = z_meas - kf_x_;

  // S = H P H^T + R = P00 + R
  double S = P00_ + r_meas_;
  if (S < 1e-9) return;

  // K = P H^T S^-1 = [P00, P10]^T / S
  double K0 = P00_ / S;
  double K1 = P10_ / S;

  // state update
  kf_x_ = kf_x_ + K0 * y;
  kf_v_ = kf_v_ + K1 * y;

  // covariance update: P = (I - K H) P
  // (I-KH) = [[1-K0, 0],
  //           [-K1,  1]]
  double P00 = P00_, P01 = P01_, P10 = P10_, P11 = P11_;

  P00_ = (1.0 - K0) * P00;
  P01_ = (1.0 - K0) * P01;
  P10_ = P10 - K1 * P00;
  P11_ = P11 - K1 * P01;
}


  void control_loop()
  {
    if (!have_image_ || !have_depth_ || !have_imu_) return;

    const rclcpp::Time now = this->now();
    const double dt = (now - prev_time_).seconds();
    if (dt <= 1e-6) return;
    prev_time_ = now;

    if (kf_inited_) {
      kf_predict(dt);
    }


    const double depth_error = target_depth_ - depth_;
    integral_depth_ += depth_error * dt;
    const double derivative_depth = (depth_error - prev_depth_error_) / dt;
    prev_depth_error_ = depth_error;

    double heave =
      kp_depth_ * depth_error +
      ki_depth_ * integral_depth_ +
      kd_depth_ * derivative_depth;
    heave = clampd(heave, -max_heave_cmd_, max_heave_cmd_);

    double roll_cmd  = kp_roll_  * (0.0 - roll_)  + kd_roll_  * (0.0 - roll_rate_);
    double pitch_cmd = kp_pitch_ * (0.0 - pitch_) + kd_pitch_ * (0.0 - pitch_rate_);
    roll_cmd  = clampd(roll_cmd,  -max_att_cmd_, max_att_cmd_);
    pitch_cmd = clampd(pitch_cmd, -max_att_cmd_, max_att_cmd_);

    double x_center = 0.0, lane_theta = 0.0;
    cv::Mat red_mask, white_mask, overlay;
    const bool lane_ok = detect_lane(image_, x_center, lane_theta, red_mask, white_mask, overlay);

    if (lane_ok) {
      if (!kf_inited_) {
        kf_reset(x_center);
      } else {
        kf_update(x_center);
      }

      x_center = kf_x_;
    } else {
      int y_draw = static_cast<int>(0.65 * image_.rows);
      cv::circle(overlay, cv::Point(static_cast<int>(kf_x_), y_draw), 6, cv::Scalar(255, 255, 0), -1);

    }


    frame_count_++;

    if (show_cv_) {
      cv::imshow("rgb", image_);
      cv::imshow("red_mask", red_mask);
      cv::imshow("white_mask", white_mask);
      cv::imshow("overlay", overlay);
      cv::waitKey(1);
    }

    if (publish_debug_) {
      if ((frame_count_ % debug_every_n_) == 0) {
        publish_dbg_images(image_, red_mask, white_mask, overlay);
      }
    }

    geometry_msgs::msg::Twist cmd;
    cmd.linear.z = heave;
    cmd.angular.x = roll_cmd;
    cmd.angular.y = pitch_cmd;

    const int W = image_.cols;
    const double img_cx = 0.5 * static_cast<double>(W);

    if (!lane_ok) {
      cmd.linear.x = forward_speed_search_;
      cmd.angular.z = clampd(search_yaw_ + (-kd_yaw_ * yaw_rate_), -max_yaw_, max_yaw_);

      cmd_pub_->publish(cmd);

      if (debug_ && (frame_count_ % debug_every_n_) == 0) {
        RCLCPP_INFO(this->get_logger(),
                    "[SEARCH] x=%.2f z=%+.2f r=%+.2f p=%+.2f yaw=%+.2f depth=%.2f",
                    cmd.linear.x, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z, depth_);
      }
      return;
    }

    last_lane_seen_time_ = now;

    const double error_px = (img_cx - x_center);

    const double yaw_track = kp_yaw_px_ * error_px;
    const double yaw_ff = -k_lane_theta_ * lane_theta;   
    const double yaw_damp = -kd_yaw_ * yaw_rate_;

    const double yaw_cmd = clampd(yaw_track + yaw_ff + yaw_damp, -max_yaw_, max_yaw_);

    cmd.linear.x = forward_speed_;
    cmd.angular.z = yaw_cmd;

    cmd_pub_->publish(cmd);

    if (debug_ && (frame_count_ % debug_every_n_) == 0) {
      RCLCPP_INFO(this->get_logger(),
                  "[LANE] x=%.2f z=%+.2f r=%+.2f p=%+.2f yaw=%+.2f err_px=%+.1f theta=%+.3f depth=%.2f",
                  cmd.linear.x, cmd.linear.z, cmd.angular.x, cmd.angular.y, cmd.angular.z,
                  error_px, lane_theta, depth_);
    }
  }

private:
  bool debug_{true};
  bool show_cv_{false};
  bool publish_debug_{true};
  int debug_every_n_{2};

  double target_depth_{1.0};
  bool depth_increases_down_{true};
  double kp_depth_{4.0}, ki_depth_{0.1}, kd_depth_{0.5}, max_heave_cmd_{2.0};

  double kp_roll_{3.0}, kd_roll_{0.8}, kp_pitch_{3.0}, kd_pitch_{0.8}, max_att_cmd_{2.0};

  double kp_yaw_px_{0.006}, kd_yaw_{0.05}, k_lane_theta_{0.6};
  double max_yaw_{1.2}, search_yaw_{0.35};
  double lane_lookahead_frac_{0.65}, lane_found_timeout_{0.5};

  double forward_speed_{0.35}, forward_speed_search_{0.05};

  bool kf_inited_{false};
  double kf_x_{0.0};      
  double kf_v_{0.0};      
  double P00_{1.0}, P01_{0.0}, P10_{0.0}, P11_{1.0};  

  double q_pos_{50.0};    
  double q_vel_{200.0};   
  double r_meas_{400.0};  


  int min_area_{300};
  double min_aspect_{2.0};
  int blur_ksize_{5};
  int morph_ksize_{5};
  int max_poles_each_color_{6};

  cv::Scalar red1_low_, red1_high_, red2_low_, red2_high_;
  cv::Scalar white_low_, white_high_;

  cv::Mat image_;
  bool have_image_{false};

  double depth_{0.0};
  bool have_depth_{false};

  double roll_{0.0}, pitch_{0.0}, yaw_{0.0};
  double roll_rate_{0.0}, pitch_rate_{0.0}, yaw_rate_{0.0};
  bool have_imu_{false};

  double integral_depth_{0.0};
  double prev_depth_error_{0.0};

  rclcpp::Time prev_time_;
  rclcpp::Time last_lane_seen_time_;

  uint64_t frame_count_{0};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<ros_gz_interfaces::msg::Altimeter>::SharedPtr alt_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_red_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_white_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dbg_overlay_pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaneNavigator>();
  rclcpp::spin(node);

  try { cv::destroyAllWindows(); } catch (...) {}
  rclcpp::shutdown();
  return 0;
}
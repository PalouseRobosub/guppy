#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "guppy_msgs/msg/transform_list.hpp"
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "rclcpp_action/rclcpp_action.hpp"
#include "guppy_msgs/action/align.hpp"

using Align = guppy_msgs::action::Align;
using GoalHandle = rclcpp_action::ServerGoalHandle<Align>;

using namespace std::chrono_literals;

//TUNE THESE VARIABLES

static constexpr double kp_f = 0.3;//KP FORWARD
static constexpr double kp_l = 0.4;//KP LATERAL
static constexpr double kp_v = 0.4;//KP VERTICAL
static constexpr double kp_y = 0.5;//KP YAW

static constexpr double td = 1.0;//DESIRED TARGETTING DISTANCE (m)
static constexpr double db_lin = 0.02;//DEADBAND LINEAR
static constexpr double db_ang = 0.3;//DEADBAND ANGULAR

static constexpr double m_lin = 0.3;//MAX LINEAR m/s CLAMP
static constexpr double m_ang = 0.5;//MAX ANGULAR rad/s CLAMP

static constexpr double timeout = 0.5;//STOP DETECTION FOR TIMEOUT (s)

static double clamp(double v, double limit){
    return std::max(-limit,std::min(limit,v));
}

static double deadband(double v, double band){
    return std::abs(v) < band ? 0.0 : v;
}

class AlignNode : public rclcpp::Node{
    public:
    AlignNode() : Node("align"){
        transforms_sub_ = this->create_subscription<guppy_msgs::msg::TransformList>("/cam/test/transforms", 10, std::bind(&AlignNode::callback, this, std::placeholders::_1));
        cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        action_server_ = rclcpp_action::create_server<Align>(this, "align",std::bind(&AlignNode::handle_goal,     this, std::placeholders::_1, std::placeholders::_2),std::bind(&AlignNode::handle_cancel,   this, std::placeholders::_1),std::bind(&AlignNode::handle_accepted, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "ALIGN NODE STARTED. TARGET DISTANCE: %.2f m", td);
    };
    private:
    void callback(guppy_msgs::msg::TransformList::UniquePtr msg){
        if (msg->transforms.empty()){
            return;
        }
        auto & tf = msg -> transforms[0];
        last_detection_time = this->now();
        has_detection = true;
        double ex = tf.transform.translation.x;//LATERAL, ASSUME + IS RIGHT
        double ey = tf.transform.translation.y;//VERTICAL, ASSUME + IS DOWN
        double ez = tf.transform.translation.z;//FORWARD (away from camera)

        //double forward_error = ez - td;
        //double lateral_error = ex;
        //double vertical_error = ey;

        auto & q = tf.transform.rotation;
        Eigen::Quaterniond quat(q.w, q.x, q.y, q.z);
        Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);
        // double yaw_error = euler[2];
        // RCLCPP_DEBUG(this->get_logger(),"tag: %s  fwd_err=%.3f  lat_err=%.3f  vert_err=%.3f  yaw_err=%.3f rad",tf.child_frame_id.c_str(), forward_error, lateral_error, vertical_error, yaw_error);

        // //P CONTROLLER
        // geometry_msgs::msg::Twist cmd;
        // cmd.linear.x = clamp(kp_f * deadband(-forward_error, db_lin),m_lin);
        // cmd.linear.y = clamp(kp_l * deadband(-lateral_error, db_lin),m_lin);
        // cmd.linear.z = clamp(kp_v * deadband(-vertical_error, db_lin),m_lin);
        // cmd.angular.z = clamp(kp_y * deadband(-yaw_error,db_ang),m_ang);
        latest_ex = tf.transform.translation.x;
        latest_ey = tf.transform.translation.y;
        latest_ez = tf.transform.translation.z;
        latest_yaw = euler[2];
        last_detection_time = this->now();
        has_detection = true;
        // cmd_vel_pub->publish(cmd);
        // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,"cmd: vx=%.2f vy=%.2f vz=%.2f wz=%.2f",cmd.linear.x, cmd.linear.y, cmd.linear.z, cmd.angular.z);
    }

    // void watchdog_callback(){
    //     if(!has_detection){
    //         return;
    //     }
    //     double elapsed = (this->now() - last_detection_time).seconds();
    //     if(elapsed > timeout){
    //         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,"NO APRIL TAG DETECTED FOR %.1f s. STOPPING.", elapsed);
    //         cmd_vel_pub->publish(geometry_msgs::msg::Twist{});
    //     }
    // }

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Align::Goal> goal){
        RCLCPP_INFO(this->get_logger(), "RECEIVED GOAL: TAG=%s DIST=%.2F",goal->tag_name.c_str(), goal->target_distance);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>) {
        stop();
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle){
        std::thread([this, goal_handle]() { execute(goal_handle); }).detach();
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle){
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<Align::Feedback>();
        auto result = std::make_shared<Align::Result>();
        rclcpp::Rate rate(10);

        while (rclcpp::ok()){
            if(goal_handle->is_canceling()){
                stop();
                result->success = false;
                result->message = "CANCELLED";
                goal_handle->canceled(result);
                return;
            }
            if(!has_detection || (this->now() - last_detection_time).seconds() > 0.5){
                stop();
                rate.sleep();
                continue;
            }
            double forward_error = latest_ez - goal->target_distance;
            double lateral_error = latest_ex;
            double vertical_error = latest_ey;
            double yaw_error = latest_yaw;
            feedback->forward_error = forward_error;
            feedback->lateral_error = lateral_error;
            feedback->vertical_error = vertical_error;
            feedback->yaw_error = yaw_error;
            goal_handle->publish_feedback(feedback);
            if(std::abs(forward_error) < success_threshold &&
            std::abs(lateral_error) < success_threshold &&
            std::abs(vertical_error) < success_threshold &&
            std::abs(yaw_error) < success_threshold){
                stop();
                result->success = true;
                result->message = "ALIGNED SUCCESSFULLY";
                goal_handle->succeed(result);
                return;
            }
            geometry_msgs::msg::Twist cmd;
            //ASSUMES Z IS FORWARD/BACKWARD, Y IS UP/DOWN, X LEFT/RIGHT
            // cmd.linear.x  = clamp(kp_f * deadband(-forward_error,  db_lin), m_lin);
            // cmd.linear.y  = clamp(kp_l * deadband(-lateral_error,  db_lin), m_lin);
            // cmd.linear.z  = clamp(kp_v * deadband(-vertical_error, db_lin), m_lin);
            // cmd.angular.z = clamp(kp_y * deadband(-yaw_error,      db_ang), m_ang);
            //ASSUMES Y IS FORWARD/BACKWARD, X IS LEFT/RIGHT, Z UP/DOWN
            cmd.linear.y  = clamp(kp_f * deadband(-forward_error,  db_lin), m_lin);  
            cmd.linear.x  = clamp(kp_l * deadband(-lateral_error,  db_lin), m_lin);  
            cmd.linear.z  = clamp(kp_v * deadband(-vertical_error, db_lin), m_lin);  
            cmd.angular.z = clamp(kp_y * deadband(-yaw_error,      db_ang), m_ang);
            cmd_vel_pub->publish(cmd);
            rate.sleep();
        }
    }

    void stop(){
        cmd_vel_pub->publish(geometry_msgs::msg::Twist{});
    }
    rclcpp::Subscription<guppy_msgs::msg::TransformList>::SharedPtr transforms_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
    //rclcpp::TimerBase::SharedPtr watchdog;
    rclcpp_action::Server<Align>::SharedPtr action_server_;
    double latest_ex = 0, latest_ey= 0, latest_ez = 0, latest_yaw = 0;
    static constexpr double success_threshold = 0.05;
    rclcpp::Time last_detection_time;
    bool has_detection = false;


};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AlignNode>());
    rclcpp::shutdown();
    return 0;
}


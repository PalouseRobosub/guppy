#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "guppy_control/chassis_controller.hpp"
#include "guppy_control/t200_interface.hpp"

using namespace std::chrono_literals;
using namespace t200_interface;
using namespace chassis_controller;

class ControlChassis : public rclcpp::Node {
public:
  ControlChassis() : Node("control_chassis") {
    thruster_interface = new T200Interface("can0", {101, 102, 103, 104, 105, 106, 107, 108});

    // setup parameters...

    ChassisController::ChassisControllerParams parameters;
    Eigen::Matrix<double, 6, 8> temp_motor_coeff_matrix;
    //                               X    Y    Z  PHI   THETA
    temp_motor_coeff_matrix <<\
      get_single_motor_coefficients(-1,   1,   0,  0,    135), \
      get_single_motor_coefficients( 1,   1,   0,  0,   -135), \
      get_single_motor_coefficients( 1,  -1,   0,  0,    -45), \
      get_single_motor_coefficients(-1,  -1,   0,  0,     45), \
      get_single_motor_coefficients( 0,   1,   0,  90,     0), \
      get_single_motor_coefficients( 1,   0,   0,  90,     0), \
      get_single_motor_coefficients( 0,  -1,   0,  90,     0), \
      get_single_motor_coefficients(-1,   0,   0,  90,     0);

    parameters.motor_coefficients = temp_motor_coeff_matrix;
    parameters.motor_lower_bounds << -49.42552, -49.42552, -49.42552, -49.42552, -49.42552, -49.42552, -49.42552, -49.42552;
    parameters.motor_upper_bounds << 64.23356, 64.23356, 64.23356, 64.23356, 64.23356, 64.23356, 64.23356, 64.23356;
    parameters.pid_gains_vel_linear = {100, 0, 0};
    parameters.pid_gains_vel_angular = {100, 0, 0};
    parameters.pid_gains_pose_linear = {100, 0, 0};
    parameters.pid_gains_pose_angular = {1, 0, 0};
    parameters.drag_coefficients << 0,0,0,0,0,0;
    parameters.drag_areas<< 1, 1, 1, 1, 1, 1;
    parameters.drag_effect_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    parameters.water_density = 1000; // kg/m^3
    parameters.robot_volume = 0.02; // m^3
    parameters.robot_mass = 20; // kg
    parameters.center_of_buoyancy << 0, 0, 0.04;
    parameters.qp_epsilon = 1e-3;
    parameters.pose_lock_deadband << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    controller = new ChassisController(parameters, thruster_interface, 100000);
    controller->start();

    // setup pub/sub/timer
    for (int i=0; i<8; i++) {
      sim_motor_publishers_[i] = this->create_publisher<std_msgs::msg::Float32>("/sim/motor_forces/m_"+std::to_string(i), 10);
    }
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&ControlChassis::odom_callback, this, std::placeholders::_1));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&ControlChassis::cmdvel_callback, this, std::placeholders::_1));


    auto timer_callback = [this]() -> void {
      auto thrusts = controller->get_motor_thrusts();
      for (int i=0; i<8; i++) {
        std_msgs::msg::Float32 thrust;
        thrust.data = (double)thrusts[i];
        sim_motor_publishers_[i].get()->publish(thrust);
      }
    };

    timer_ = this->create_wall_timer(10ms, timer_callback);

    RCLCPP_INFO(this->get_logger(), "setup publisher and subscriber");
  }

  ~ControlChassis() {
    controller->stop();
    delete controller;
    delete thruster_interface;
  }

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    controller->update_current_state(msg);
  }

  void cmdvel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    controller->update_desired_state(msg);
  }

  /*
    @brief get the motor coefficients from a thruster's pose
    @param x the x loctaion in meters of the thruster
    @param y the y loctaion in meters of the thruster
    @param z the z loctaion in meters of the thruster
    @param phi the phi of the thruster in spherical coordinates (degrees, positive up)
    @param theta the theta of the thruster in spherical coordinates (degrees)
  */
  Eigen::Vector<double, 6> get_single_motor_coefficients(double x, double y, double z, double phi, double theta) {
    Eigen::Vector<double, 6> out;

    // convert to rads
    double p = (90 - phi) * (M_PI / 180);
    double t = (270 + theta) * (M_PI / 180);

    // calculate to reuse
    double sinp = sin(p);
    double sint = sin(t);
    double cost = cos(t);
    double cosp = cos(p);

    out <<\
        sinp * cost,                  \
        sinp * sint,                  \
        cosp,                         \
        (z*sinp*sint) - (y*cosp),     \
        (x*cosp) - (z*sinp*cost),     \
        (y*sinp*cost) - (x*sinp*sint);
    return out;
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_motor_publishers_[8];
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  T200Interface* thruster_interface;
  ChassisController* controller;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlChassis>());
  rclcpp::shutdown();
  return 0;
}
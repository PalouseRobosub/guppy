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
    thruster_interface = new T200Interface("can0", {0, 1, 2, 3, 4, 5, 6, 7});

    ChassisController::ChassisControllerParams parameters;
    parameters.motor_coefficients <<\
     1,   1,    1,    1.5,    1,    2,    3,    4, \
		 4,   0,    0,    2,      1,    2,    3,    4, \
		 4,   0,    0,    2,      1,    2,    3,    4, \
		 4,   0,    0,    2,      3,    4,    5,    6, \
		 4,   0,    0,    2,      6,    7,    8,    9, \
		 3.5, 3,    3,    2.5,    1,    1,    1,    1;
    parameters.motor_lower_bounds << -1, -1, -1, -1, -1, -1, -1, -1;
    parameters.motor_upper_bounds << 1, 1, 1, 1, 1, 1, 1, 1;
    parameters.pid_gains_vel_linear = control_toolbox::Pid::Gains(10000, 0, 0, 100, -100, ChassisController::antiwindup_strat);
    parameters.pid_gains_vel_angular = control_toolbox::Pid::Gains(10000, 0, 0, 100, -100, ChassisController::antiwindup_strat);
    parameters.pid_gains_pose_linear = control_toolbox::Pid::Gains(10000, 0, 0, 100, -100, ChassisController::antiwindup_strat);
    parameters.pid_gains_pose_angular = control_toolbox::Pid::Gains(10000, 0, 0, 100, -100, ChassisController::antiwindup_strat);
    parameters.drag_coefficients << 1, 1, 1, 1, 1, 1;
    parameters.drag_areas<< 1, 1, 1, 1, 1, 1;
    parameters.drag_effect_matrix = Eigen::Matrix<double, 6, 6>::Identity();
    parameters.water_density = 1000; // kg/m^3
    parameters.robot_volume = 2; // m^3
    parameters.center_of_buoyancy << 0, 0, 1;
    parameters.qp_epsilon = 1e-3;
    parameters.pose_lock_deadband << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;

    controller = new ChassisController(parameters, thruster_interface, 1000);
    controller->start();

    // setup pub/sub/timer
    for (int i=0; i<6; i++) {
      sim_motor_publishers_[i] = this->create_publisher<std_msgs::msg::Float32>("/sim/motor_forces/m_"+std::to_string(i), 10);
    }
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&ControlChassis::odom_callback, this, std::placeholders::_1));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&ControlChassis::cmdvel_callback, this, std::placeholders::_1));


    auto timer_callback = [this]() -> void {
      auto thrusts = controller->get_motor_thrusts();
      for (int i=0; i<6; i++) {
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

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_motor_publishers_[6];
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
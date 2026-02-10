#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "guppy_control/chassis_controller.hpp"
#include "guppy_control/t200_interface.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using namespace t200_interface;
using namespace chassis_controller;

class ControlChassis : public rclcpp::Node {
public:
  ControlChassis() : Node("control_chassis") {
    thruster_interface = new T200Interface("can0", {101, 102, 103, 104, 105, 106, 107, 108});

    this->declare_parameter<std::vector<double>>("motor_coeff_matrix"); // flattened bc nested vector wouldn't work, size of 6 * N_MOTORS
    this->declare_parameter<std::vector<double>>("motor_lower_bounds"); // size of N_MOTORS
    this->declare_parameter<std::vector<double>>("motor_upper_bounds"); // size of N_MOTORS
    // this->declare_parameter<std::vector<std::vector<double, 6>, 6>>("axis_weight_matrix"); // just an identity matrix so won't need to be updated?
    this->declare_parameter<std::vector<double>>("pid_gains_vel_linear"); // size of 3
    this->declare_parameter<std::vector<double>>("pid_gains_vel_angular"); // size of 3
    this->declare_parameter<std::vector<double>>("pid_gains_pose_linear"); // size of 3
    this->declare_parameter<std::vector<double>>("pid_gains_pose_angular"); // size of 3
    this->declare_parameter<std::vector<double>>("pose_lock_deadband"); // size of 6
    this->declare_parameter<std::vector<double>>("drag_coefficients"); // size of 3
    this->declare_parameter<std::vector<double>>("drag_areas"); // size of 3
    this->declare_parameter<std::vector<double>>("drag_effect_matrix"); // only has 5 parameters? flattened size of 6 * 6
    this->declare_parameter<double>("water_density"); // does this need to be changed?
    this->declare_parameter<double>("robot_volume");
    this->declare_parameter<double>("robot_mass");
    this->declare_parameter<std::vector<double>>("center_of_buoyancy"); // size of 3
    this->declare_parameter<double>("qp_epsilon");

    // setup parameters...

    ChassisController::ChassisControllerParams parameters;

    parameters.drag_effect_matrix = Eigen::Matrix<double, 6, 6>::Identity();

    this->param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);

    auto param_callback = [this](const rcl_interfaces::msg::ParameterEvent & parameter_event) {
      RCLCPP_INFO(this->get_logger(), "Recieved parameter event from node for \"%s\"!", parameter_event.node.c_str());

      for (const auto& param : parameter_event.changed_parameters) {
        const std::string name = rclcpp::Parameter::from_parameter_msg(param).get_name();
        
        const auto value = rclcpp::Parameter::from_parameter_msg(param);

        auto controller_params = this->controller->get_params();
        if (name == "motor_coeff_matrix") controller_params.motor_coefficients = to_eigen_matrix<6, N_MOTORS>(value.as_double_array());
        else if (name == "motor_lower_bounds") controller_params.motor_lower_bounds = to_eigen_vec<N_MOTORS>(value.as_double_array());
        else if (name == "motor_upper_bounds") controller_params.motor_upper_bounds = to_eigen_vec<N_MOTORS>(value.as_double_array());
        //else if (name == "axis_weight_matrix") // just an identity matrix, cannot change
        else if (name == "pid_gains_vel_linear") controller_params.pid_gains_vel_linear = value.as_double_array();
        else if (name == "pid_gains_vel_angular") controller_params.pid_gains_vel_angular = value.as_double_array();
        else if (name == "pid_gains_pose_linear") controller_params.pid_gains_pose_linear = value.as_double_array();
        else if (name == "pid_gains_pose_angular") controller_params.pid_gains_pose_angular = value.as_double_array();
        else if (name == "pose_lock_deadband") controller_params.pose_lock_deadband = to_eigen_vec<6>(value.as_double_array());
        else if (name == "drag_coefficients") controller_params.drag_coefficients = to_eigen_vec<6>(value.as_double_array());
        else if (name == "drag_areas") controller_params.drag_areas = to_eigen_vec<6>(value.as_double_array());
        else if (name == "drag_effect_matrix") controller_params.drag_effect_matrix = to_eigen_matrix<6, 6>(value.as_double_array());
        else if (name == "water_density") controller_params.water_density = value.as_double();
        else if (name == "robot_volume") controller_params.robot_volume = value.as_double();
        else if (name == "robot_mass") controller_params.robot_mass = value.as_double();
        else if (name == "center_of_buoyancy") {
          auto& vec = value.as_double_array();
          controller_params.center_of_buoyancy = Eigen::Vector3<double>(vec[0], vec[1], vec[2]);
        } else if (name == "qp_epsilon") controller_params.qp_epsilon = value.as_double();
      }
    };
    this->param_event_callback_handle_ = param_subscriber_->add_parameter_event_callback(param_callback);

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

  template<int N> Eigen::Vector<double, N> to_eigen_vec(const std::vector<double>& vec) {
    if (vec.size() != N) {
      RCLCPP_ERROR(this->get_logger(), "Bad vector parameter passed to controller!");
      throw std::runtime_error("Bad parameter passed to controller.");
    }

    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(vec.data());
  }

  template<int R, int C> Eigen::Matrix<double, R, C> to_eigen_matrix(const std::vector<double>& vec) {
    if (vec.size() != R * C) {
      RCLCPP_ERROR(this->get_logger(), "Bad matrix parameter passed to controller!");
      throw std::runtime_error("Bad matric parameter passed to controller.");
    }

    return Eigen::Map<const Eigen::Matrix<double, R, C, Eigen::RowMajor>>(vec.data());
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_motor_publishers_[8];
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_event_callback_handle_;

  T200Interface* thruster_interface;
  ChassisController* controller;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ControlChassis>());

  rclcpp::shutdown();
  return 0;
}
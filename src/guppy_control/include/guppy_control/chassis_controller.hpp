#ifndef GUPPY_CHASSIS_CONTROLLER_H
#define GUPPY_CHASSIS_CONTROLLER_H

#define GRAVITY 9.80665
#define N_MOTORS 8

#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <atomic>

#include <Eigen/Core>
#include <proxsuite/proxqp/dense/dense.hpp>
#include <control_toolbox/control_toolbox/pid.hpp>
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "guppy_control/t200_interface.hpp"

namespace chassis_controller {

using namespace proxsuite::proxqp;
using namespace t200_interface;
using namespace std::chrono_literals;
using namespace std::chrono;

class ChassisController {
    typedef struct chassis_controller_params_ {
        // motor setup
        Eigen::Matrix<double, 6, N_MOTORS> motor_coefficients;
        Eigen::Vector<double, N_MOTORS> motor_lower_bounds;
        Eigen::Vector<double, N_MOTORS> motor_upper_bounds;

        // control parameters
        Eigen::Matrix<double, 6, 6> axis_weights = Eigen::Matrix<double, 6, 6>::Identity();
        control_toolbox::Pid::Gains pid_gains_vel_linear;
        control_toolbox::Pid::Gains pid_gains_vel_angular;

        // robot setup
        Eigen::Vector<double, 6> drag_coefficients;
        Eigen::Vector<double, 6> drag_areas;
        Eigen::Matrix<double, 6, 6> drag_effect_matrix;
        double water_density = 1000; // kg/m^3
        double robot_volume; // m^3
        Eigen::Vector3<double> center_of_buoyancy;

        // qp solver
        double qp_epsilon = 1e-2;
    } ChassisControllerParams;


    T200Interface interface_;

    Eigen::Vector<double, 6> current_velocity_state_;
    Eigen::Vector<double, 6> desired_velocity_state_;
    Eigen::Quaternion<double> current_orientation_state_;
    Eigen::Vector3d current_position_state_;

    Eigen::Vector<double, N_MOTORS> motor_forces_;

    ChassisControllerParams params_;
    control_toolbox::Pid velocity_pid[6];

    dense::QP<double> qp_;

    int dt_ms_;
    std::atomic<bool> THREAD_RUNNING_;
    std::thread control_thread_;

    // private methods
    Eigen::Vector<double, N_MOTORS> allocate_thrust(Eigen::Vector<double, 6> local_wrench);
    bool control_loop();
    void loop_runner();

public:
    ChassisController(ChassisControllerParams parameters, T200Interface hw_interface, int dt_ms=10);
    ~ChassisController();

    void update_current_state(nav_msgs::msg::Odometry::SharedPtr msg);
    void update_desired_state(geometry_msgs::msg::Twist::SharedPtr msg);
    void update_parameters(ChassisControllerParams parameters);

    void start();
    void stop();

    Eigen::Vector<double, N_MOTORS> get_motor_thrusts();
};

}

#endif
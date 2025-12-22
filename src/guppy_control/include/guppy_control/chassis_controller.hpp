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
public:
    // strange chatgpt magic to bypass the deprecated constructor... 
    inline static const control_toolbox::AntiWindupStrategy antiwindup_strat = [] {
        control_toolbox::AntiWindupStrategy a;
        a.set_type("back_calculation");
        return a;
    }();
    // end weird magic

    typedef enum orientation_lock_state_struct_ {
        ALL_LOCKED          = 0b111,
        ROLL_PITCH_LOCK     = 0b110,
        ROLL_YAW_LOCK       = 0b101,
        ROLL_LOCK           = 0b100,
        PITCH_YAW_LOCK      = 0b011,
        PITCH_LOCK          = 0b010,
        YAW_LOCK            = 0b001,
        ALL_FREE            = 0b000
    } OrientationLockState;

    typedef struct chassis_controller_params_ {
        // motor setup
        Eigen::Matrix<double, 6, N_MOTORS> motor_coefficients;
        Eigen::Vector<double, N_MOTORS> motor_lower_bounds;
        Eigen::Vector<double, N_MOTORS> motor_upper_bounds;

        // control parameters
        Eigen::Matrix<double, 6, 6> axis_weight_matrix = Eigen::Matrix<double, 6, 6>::Identity();
        control_toolbox::Pid::Gains pid_gains_vel_linear   = control_toolbox::Pid::Gains(1, 0, 0, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiwindup_strat);
        control_toolbox::Pid::Gains pid_gains_vel_angular  = control_toolbox::Pid::Gains(1, 0, 0, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiwindup_strat);
        control_toolbox::Pid::Gains pid_gains_pose_linear  = control_toolbox::Pid::Gains(1, 0, 0, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiwindup_strat);
        control_toolbox::Pid::Gains pid_gains_pose_angular = control_toolbox::Pid::Gains(1, 0, 0, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiwindup_strat);
        Eigen::Vector<double, 6> pose_lock_deadband;

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

    ChassisController(ChassisControllerParams parameters, T200Interface* hw_interface, int dt_us=500);
    ~ChassisController();

    void update_current_state(nav_msgs::msg::Odometry::SharedPtr msg);
    void update_desired_state(geometry_msgs::msg::Twist::SharedPtr msg);
    void update_parameters(ChassisControllerParams parameters);

    void start();
    void stop();

    Eigen::Vector<double, N_MOTORS> get_motor_thrusts();
private:
    T200Interface* interface_;

    Eigen::Vector<double, 6> current_velocity_state_;
    Eigen::Quaternion<double> current_orientation_state_;
    Eigen::Vector3d current_position_state_;

    Eigen::Quaternion<double> desired_orientation_state_;
    Eigen::Vector3d desired_position_state_;
    Eigen::Vector<double, 6> desired_velocity_state_;

    OrientationLockState current_orientation_lock_ = ALL_FREE;

    Eigen::Vector<double, N_MOTORS> motor_forces_;

    ChassisControllerParams params_;
    std::vector<control_toolbox::Pid> velocity_pid;
    std::vector<control_toolbox::Pid> pose_pid;

    dense::QP<double> qp_;

    int dt_us_;
    std::atomic<bool> THREAD_RUNNING_;
    std::thread control_thread_;

    // private methods
    Eigen::Vector<double, N_MOTORS> allocate_thrust(Eigen::Vector<double, 6> local_wrench);
    Eigen::Vector3d calculate_rotational_nudge();
    void control_loop();
    void loop_runner();
};

}

#endif
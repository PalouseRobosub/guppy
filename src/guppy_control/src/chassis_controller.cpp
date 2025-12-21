#include "guppy_control/chassis_controller.hpp"
#include "chassis_controller.hpp"


namespace chassis_controller {

ChassisController::ChassisController(ChassisControllerParams parameters, T200Interface hw_interface, int dt_ms)
  : qp_(N_MOTORS, 0, N_MOTORS), interface_(hw_interface), dt_ms_(dt_ms), THREAD_RUNNING_(false) {
  
  qp_.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
  qp_.settings.verbose = false;

  qp_.init(std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);

  update_parameters(parameters);
}

ChassisController::~ChassisController() {
  if (THREAD_RUNNING_.load()) {
    THREAD_RUNNING_.store(false);
    if (control_thread_.joinable()) {
      control_thread_.join();
    }
  }
}

bool ChassisController::control_loop() {
  auto desired_squared = desired_velocity_state_.cwiseAbs() * desired_velocity_state_;
  
  // calculate drag effect on sub
  auto drag_plain = params_.drag_coefficients * params_.water_density * desired_squared * params_.drag_areas;
  auto drag_wrench = params_.drag_effect_matrix * drag_plain;

  // calculate gravity effect on sub
  Eigen::Vector3d gravity_force;
  gravity_force << 0, 0, -GRAVITY;
  gravity_force = current_orientation_state_ * gravity_force;
  Eigen::Vector<double, 6> gravity_wrench;
  gravity_wrench << gravity_force, 0, 0, 0;

  // calculate buoyant effect on sub
  Eigen::Vector3d buoyancy_force;
  buoyancy_force << 0, 0, (params_.water_density * params_.robot_volume * GRAVITY);
  buoyancy_force = current_orientation_state_ * buoyancy_force;

  Eigen::Vector3d buoyancy_torque;
  buoyancy_torque << 0, 0, 0;

  Eigen::Vector<double, 6> buoyancy_wrench;
  gravity_wrench << buoyancy_force, buoyancy_torque;

  // calculate total feedforward
  Eigen::Vector<double, 6> feedforward = (drag_wrench + buoyancy_wrench + gravity_wrench) * -1;

  // calculate PID of current error
  Eigen::Vector<double, 6> feedback;
  for (int i=0; i<6; i++) {
    feedback[i] = velocity_pid[i].compute_command(desired_velocity_state_[i] - current_velocity_state_[i], (dt_ms_ / 1000.0));
  }

  // write total wrench to the sub
  auto local_wrench = feedforward + feedback;
  motor_forces_ = allocate_thrust(local_wrench);
  bool success = interface_.write(motor_forces_);

  return success;
}

Eigen::Vector<double, N_MOTORS> ChassisController::allocate_thrust(Eigen::Vector<double, 6> local_wrench) {  
  Eigen::VectorXd g = - ( params_.motor_coefficients.transpose() * local_wrench );

  qp_.update(std::nullopt, g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  qp_.solve();

  return qp_.results.x;
}

void ChassisController::update_current_state(nav_msgs::msg::Odometry::SharedPtr msg) {
  auto ros_odom = msg.get();
  auto ros_quat = ros_odom->pose.pose.orientation;
  auto ros_pos = ros_odom->pose.pose.position;
  auto ros_twist = ros_odom->twist.twist;

  Eigen::Vector<double, 6> new_current_vel;
  new_current_vel << \
    ros_twist.linear.x, \
    ros_twist.linear.y, \
    ros_twist.linear.z, \
    ros_twist.angular.x, \
    ros_twist.angular.y, \
    ros_twist.angular.z;
  this->current_velocity_state_ = new_current_vel;

  Eigen::Quaterniond quat(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z);
  this->current_orientation_state_ = quat;

  Eigen::Vector3d new_current_pos;
  new_current_pos << \
    ros_pos.x, \
    ros_pos.y, \
    ros_pos.z;
  this->current_position_state_= new_current_pos;

}

void ChassisController::update_desired_state(geometry_msgs::msg::Twist::SharedPtr msg) {
  auto ros_twist = msg.get();

  Eigen::Vector<double, 6> new_desired_state;
  new_desired_state << \
    ros_twist->linear.x, \
    ros_twist->linear.y, \
    ros_twist->linear.z, \
    ros_twist->angular.x, \
    ros_twist->angular.y, \
    ros_twist->angular.z;

  this->desired_velocity_state_ = new_desired_state;
}


void ChassisController::update_parameters(ChassisControllerParams parameters) {
  this->params_ = parameters;

  Eigen::MatrixXd qp_A = params_.motor_coefficients;
  Eigen::MatrixXd qp_H = qp_A.transpose() * qp_A;

  Eigen::MatrixXd qp_C = Eigen::MatrixXd::Identity(N_MOTORS, N_MOTORS);

  qp_.settings.eps_abs = params_.qp_epsilon; // convergence amount
  qp_.update(qp_H, std::nullopt, std::nullopt, std::nullopt, qp_C, params_.motor_lower_bounds, params_.motor_upper_bounds);

  this->velocity_pid[0].set_gains(params_.pid_gains_vel_linear);
  this->velocity_pid[1].set_gains(params_.pid_gains_vel_linear);
  this->velocity_pid[2].set_gains(params_.pid_gains_vel_linear);
  this->velocity_pid[3].set_gains(params_.pid_gains_vel_angular);
  this->velocity_pid[4].set_gains(params_.pid_gains_vel_angular);
  this->velocity_pid[5].set_gains(params_.pid_gains_vel_angular);
}

Eigen::Vector<double, N_MOTORS> ChassisController::get_motor_thrusts() {
  return this->motor_forces_;
}

void ChassisController::loop_runner() {
  interface_.initialize();
  while (THREAD_RUNNING_.load()) {
    system_clock::time_point next_wake = high_resolution_clock::now() + milliseconds(dt_ms_);
    auto start = high_resolution_clock::now();

    control_loop();

    auto stop = high_resolution_clock::now();
    microseconds duration_us = duration_cast<microseconds>(stop - start);

    std::cout << duration_us.count() << " us" << std::endl;

    std::this_thread::sleep_until(next_wake);
  }
  interface_.shutdown();
}

void ChassisController::start() {
  THREAD_RUNNING_.store(true);
  std::thread(&ChassisController::loop_runner, this);
}

void ChassisController::stop() {
  THREAD_RUNNING_.store(false);
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

}  // namespace chassis_controller

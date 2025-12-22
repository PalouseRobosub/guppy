#include "guppy_control/chassis_controller.hpp"


namespace chassis_controller {

void ChassisController::control_loop() {
  Eigen::Vector<double, 6> desired_squared = desired_velocity_state_.cwiseAbs().array() * desired_velocity_state_.array();
  
  // calculate drag effect on sub
  Eigen::Vector<double, 6> drag_plain = params_.drag_coefficients.array() * params_.water_density * desired_squared.array() * params_.drag_areas.array();
  auto drag_wrench = params_.drag_effect_matrix * drag_plain;

  // calculate gravity effect on sub
  Eigen::Vector3d gravity_force;
  gravity_force << 0, 0, -GRAVITY;
  gravity_force = current_orientation_state_ * gravity_force;
  Eigen::Vector<double, 6> gravity_wrench;
  gravity_wrench << gravity_force, 0, 0, 0;

  // calculate buoyant effect on sub
  Eigen::Vector3d buoyancy_force; buoyancy_force << 0, 0, params_.water_density * params_.robot_volume * GRAVITY;
  Eigen::Vector3d r_vec = current_orientation_state_ * params_.center_of_buoyancy;
  Eigen::Vector3d buoyancy_torque = r_vec.cross(buoyancy_force);
  Eigen::Vector3d buoyancy_force_rotated = current_orientation_state_.inverse() * buoyancy_force;
  Eigen::Vector<double, 6> buoyancy_wrench; buoyancy_wrench << buoyancy_force_rotated, buoyancy_torque;

  // std::cout << "buoyancy_wrench: " << buoyancy_wrench.transpose() << std::endl;
  // std::cout << "gravity_wrench: " << gravity_wrench.transpose() << std::endl;
  // std::cout << "drag_wrench: " << drag_wrench.transpose() << std::endl;

  // calculate total feedforward
  Eigen::Vector<double, 6> feedforward = -(drag_wrench + buoyancy_wrench + gravity_wrench);

  // calculate PID of current velocity error
  Eigen::Vector<double, 6> velocity_feedback;
  for (int i=0; i<6; i++) {
    velocity_feedback[i] = velocity_pid[i].compute_command(desired_velocity_state_[i] - current_velocity_state_[i], (dt_us_ / 1000000.0));
  }

  // std::cout << "velocity_feedback: " << velocity_feedback.transpose() << std::endl;


  // calculate position and orientation pid

  Eigen::Vector<double, 6> added_pose_nudge;
  Eigen::Vector3d position_nudge = Eigen::Vector3d::Zero();

  // positions...
  for (int i=0; i<3; i++) {
    if (abs(desired_velocity_state_[i]) < params_.pose_lock_deadband[i]) {
      position_nudge[i] = pose_pid[i].compute_command(desired_position_state_[i] - current_position_state_[i], (dt_us_ / 1000000.0));
    } else {
      desired_position_state_[i] = current_position_state_[i];
    }
  }

  // orientation...
  Eigen::Vector3d rotational_nudge = calculate_rotational_nudge();
  added_pose_nudge << position_nudge, rotational_nudge;

  // std::cout << "pose_nudge: " << added_pose_nudge.transpose() << std::endl;
  // std::cout << std::endl;

  // write total wrench to the sub
  auto local_wrench = feedforward + velocity_feedback + added_pose_nudge;
  // std::cout << "local_wrench: " << local_wrench.transpose() << std::endl;
  motor_forces_ = allocate_thrust(local_wrench);
  // std::cout << std::endl;

  Eigen::Vector<double, N_MOTORS> motor_throttles;

  for (int i=0; i<N_MOTORS; i++) {
    double max_in_dir = motor_forces_[i] < 0 ? params_.motor_lower_bounds[i] : params_.motor_upper_bounds[i];
    motor_throttles[i] = motor_forces_[i] / abs(max_in_dir);
  }

  // bool success = 
  interface_->write(motor_throttles);

  // return success;
}

Eigen::Vector3d ChassisController::calculate_rotational_nudge() {
  int new_orientation_lock = ALL_FREE; // == 0

  if (abs(desired_velocity_state_[3]) < params_.pose_lock_deadband[3]) new_orientation_lock |= ROLL_LOCK;
  if (abs(desired_velocity_state_[4]) < params_.pose_lock_deadband[4]) new_orientation_lock |= PITCH_LOCK;
  if (abs(desired_velocity_state_[5]) < params_.pose_lock_deadband[5]) new_orientation_lock |= YAW_LOCK;
  
  if ((int)current_orientation_lock_ != new_orientation_lock) {
    desired_orientation_state_ = current_orientation_state_;
    current_orientation_lock_ = (OrientationLockState)new_orientation_lock;
  }

  // std::cout << "lock_state: " << (int)new_orientation_lock << std::endl;
  // std::cout << "old_state: " << (int)current_orientation_lock_ << std::endl;

  Eigen::Quaternion q_err = current_orientation_state_.inverse() * desired_orientation_state_;
  Eigen::Vector3d axis_err = q_err.vec();

  if (q_err.w() < 0) axis_err = -axis_err;

  // std::cout << "c: " << current_orientation_state_.vec().transpose() << std::endl;
  // std::cout << "d: " << "w: " << desired_orientation_state_.w() << " " << desired_orientation_state_.vec().transpose() << std::endl;
  // std::cout << "axis_err: " << axis_err.transpose() << std::endl;

  Eigen::Vector3d output_nudge = Eigen::Vector3d::Zero();

  if (ROLL_LOCK & current_orientation_lock_) output_nudge[0] = -1 * pose_pid[3].compute_command(axis_err[0], (dt_us_ / 1000000.0));
  if (PITCH_LOCK & current_orientation_lock_) output_nudge[1] = -1 * pose_pid[4].compute_command(axis_err[1], (dt_us_ / 1000000.0));
  if (YAW_LOCK & current_orientation_lock_) output_nudge[2] = -1 * pose_pid[5].compute_command(axis_err[2], (dt_us_ / 1000000.0));

  return output_nudge;
}

ChassisController::ChassisController(ChassisControllerParams parameters, T200Interface* hw_interface, int dt_us)
  : params_(parameters), qp_(N_MOTORS, 0, N_MOTORS), interface_(hw_interface), dt_us_(dt_us),\
    THREAD_RUNNING_(false), desired_orientation_state_(1, 0, 0, 0), current_orientation_state_(1, 0, 0, 0) {
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

Eigen::Vector<double, N_MOTORS> ChassisController::allocate_thrust(Eigen::Vector<double, 6> local_wrench) { 
  Eigen::VectorXd qp_g = - ( params_.motor_coefficients.transpose() * params_.axis_weight_matrix * local_wrench );

  qp_.update(std::nullopt, qp_g, std::nullopt, std::nullopt, std::nullopt, std::nullopt, std::nullopt);
  qp_.solve();

  qp_.settings.initial_guess = proxsuite::proxqp::InitialGuessStatus::WARM_START_WITH_PREVIOUS_RESULT;
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
  Eigen::MatrixXd qp_H = qp_A.transpose() * params_.axis_weight_matrix * qp_A;
  Eigen::MatrixXd qp_C = Eigen::MatrixXd::Identity(N_MOTORS, N_MOTORS);

  qp_.settings.eps_abs = params_.qp_epsilon; // convergence amount
  qp_.settings.initial_guess = InitialGuessStatus::NO_INITIAL_GUESS;
  qp_.settings.verbose = false;
  qp_.settings.compute_timings = true;

  qp_.init(qp_H, std::nullopt, std::nullopt, std::nullopt, qp_C, params_.motor_lower_bounds, params_.motor_upper_bounds);

  this->velocity_pid[0].set_gains(params_.pid_gains_vel_linear);
  this->velocity_pid[1].set_gains(params_.pid_gains_vel_linear);
  this->velocity_pid[2].set_gains(params_.pid_gains_vel_linear);
  this->velocity_pid[3].set_gains(params_.pid_gains_vel_angular);
  this->velocity_pid[4].set_gains(params_.pid_gains_vel_angular);
  this->velocity_pid[5].set_gains(params_.pid_gains_vel_angular);

  this->pose_pid[0].set_gains(params_.pid_gains_pose_linear);
  this->pose_pid[1].set_gains(params_.pid_gains_pose_linear);
  this->pose_pid[2].set_gains(params_.pid_gains_pose_linear);
  this->pose_pid[3].set_gains(params_.pid_gains_pose_angular);
  this->pose_pid[4].set_gains(params_.pid_gains_pose_angular);
  this->pose_pid[5].set_gains(params_.pid_gains_pose_angular);
}

Eigen::Vector<double, N_MOTORS> ChassisController::get_motor_thrusts() {
  return this->motor_forces_;
}

void ChassisController::loop_runner() {
  interface_->initialize();

  int min_us = 99999;
  int max_us = 0;
  while (THREAD_RUNNING_.load()) {
    auto next_wake = steady_clock::now() + microseconds(dt_us_);
    auto start = high_resolution_clock::now();

    control_loop();

    auto stop = high_resolution_clock::now();
    int duration_us = duration_cast<microseconds>(stop - start).count();
    std::cout << duration_us << " us total loop time" << std::endl;
    if (duration_us < min_us) min_us = duration_us;
    if (duration_us > max_us) max_us = duration_us;

    std::this_thread::sleep_until(next_wake);
  }
  interface_->shutdown();

  std::cout << "\t\t min_us:" << min_us << "\tmax_us: " << max_us << std::endl;
}

void ChassisController::start() {
  THREAD_RUNNING_.store(true);
  control_thread_ = std::thread(&ChassisController::loop_runner, this);
}

void ChassisController::stop() {
  THREAD_RUNNING_.store(false);
  if (control_thread_.joinable()) {
    control_thread_.join();
  }
}

}  // namespace chassis_controller

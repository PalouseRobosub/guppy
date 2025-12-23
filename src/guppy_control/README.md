# guppy_control package
Handles control of the chassis and thrusters (and more, such as the torpedos and arm, eventually).

## Chassis Controller
### `control_chassis` node
#### Subscriptions:
- `/cmd_vel` a `Twist` of the commanded velocity in the local frame
- `/odom` an `Odometry` object which contains the sub's position and velocity information
#### Publishing:
- `/sim/motor_forces/m_{0..7}` `Float32`s of the motor thrusts in Newtons for the sim.
- Transmits to the CAN bus.

### T200 Interface
This class interfaces with the T200 thrusters directly via the CAN bus. It does not use the guppy_can TX service, as it needs to maintain close to "real time" runtime (even though we aren't using an RT kernel, we should try and be close if even possible).

It's constructed with an interface name and an ordered list of CAN ids, and then `write()` is called with the throttle values (-1 to 1, not in Newtons).

### Chassis Control Loop

#### Overview
The `ChassisController` class handles the "closeish to real time" control loop and thrust allocation. It accepts a custom struct of ChassisControllerParameters to configure it:

```c++
typedef struct chassis_controller_params_ {
        
        /* A 6XN_MOTORS matrix of coefficients corresponding to each motor */
        Eigen::Matrix<double, 6, N_MOTORS> motor_coefficients;

        /* A vector of ordered motor lower bounds in Newtons of thrust. */
        Eigen::Vector<double, N_MOTORS> motor_lower_bounds;

        /* A vector of ordered motor upper bounds in Newtons of thrust. */
        Eigen::Vector<double, N_MOTORS> motor_upper_bounds;

        // control parameters
        /* A diagonal matrix of weights between Fx,Fy,Fz,Tx,Ty,Tz in the QP problem */
        Eigen::Matrix<double, 6, 6> axis_weight_matrix = Eigen::Matrix<double, 6, 6>::Identity();

         /* PID Gains for linear velocity feedback control */
        std::vector<double> pid_gains_vel_linear   = {1, 0, 0};

        /* PID Gains for angular velocity feedback control */
        std::vector<double> pid_gains_vel_angular  = {1, 0, 0};

        /* PID Gains for linear position feedback control */
        std::vector<double> pid_gains_pose_linear  = {1, 0, 0};

        /* PID Gains for angular orientation feedback control */
        std::vector<double> pid_gains_pose_angular = {1, 0, 0};

        /* Six deadband values for when an axis should be considered locked */
        Eigen::Vector<double, 6> pose_lock_deadband;

        // robot setup
        /* Drag coefficients for movement in all six axes */
        Eigen::Vector<double, 6> drag_coefficients;

        /* Drag areas of all six "axes" */
        Eigen::Vector<double, 6> drag_areas;

        /*
            A 6x6 matrix which is multiplied by drag force to predict offsets in movement
            caused by drag moment arms. For example:

                drag in:      Fx  Fy  Fz  Tx  Ty  Tz    
                           x  1   0   0   0   0   0
                           y  0   1   0   0   0   0
            causes extra:  z  0   0   1   0   0   0
                           r  0   0   0   1   0   0
                           p  0   0   0   0   1   0
                           y  0   .2  0   0   0   1

            The above matrix would mean that pitch is changed by 0.2Fx movement.
        */
        Eigen::Matrix<double, 6, 6> drag_effect_matrix;

        /* water density in kg/m^3 */
        double water_density = 1000; // kg/m^3

        /* robot volume in m^3 */
        double robot_volume; // m^3

        /* robot mass in kg */
        double robot_mass; // kg

        /* center of buoyancy arm from CM (in meters)*/
        Eigen::Vector3<double> center_of_buoyancy;

        // qp solver
        /* QP convergence epsilon */
        double qp_epsilon = 1e-2;
    } ChassisControllerParams;
```

It also gets handed a pointer to the T200 Interface for control, and a `dt_us` timestep parameter. Control loop frequency in Hertz: `Hz = 1000000 / dt_us`.

#### Math and Control Theory
The controller handles the following:
- **Feedforward:**
    - **Gravity and Buoyancy Force Control:** offsetting the restoring forces of gravity and buoyancy. The sub should have positive buoyancy, which means this generally power the vertical motors to keep the sub down. However, this also handles buoyancy when it is rotated, and vectors accordingly.
    - **Buoyancy Torque Control**: When upside down, the sub will naturally want to flip back to upright because the center of buoyancy is above the center of gravity. The code seeks to counteract this torque.
    - **Drag Force**: At a constant velocity, the drag force encountered is equal to the acting force, and increases quadratically with velocity. To counteract this drag, an equal force based on the desired velocity is calculated.
    - **Drag Moments**: Because the center of pressure is often not coaxial with the center of mass during movement, movements can also cause torques on the sub! This is fixed by multiplying a drag effect matrix with the drag vector to adjust the torques and forces accordingly.
- **Feedback:**
    - **Velocity PID**: While the drag feedforward can work for constant velocities, to accelerate between two different velocities (including from a stationary position), we run two PID controllers (one for angular and one for linear) on the velocity error to handle this acceleration and any small innacuracies that may arise.
    - **Position PID**: When velocity is 0 (or belowe a certain deadband value), the position druft should be zero as well, although there is a chance that it will drift slower than the velocity loop can handle. Because of this, we run a position controller on the linear pose if the velocity on each axis is belowe a deadband. This is part of the "pose nudge."
    - **Angular PID**: Similar to position, we only run this when angular velocity on each axis is below deadband values. We then calculate quaternion axis error between the last saved state, and run a PID controller on that. This is also part of the "pose nudge."
- **Thrust Allocation**: Finally, after the feedforward, feedback, and pose nudge terms are summed, they are passed into a Quadratic Programming (QP) solver using the Proxsuite library. This solves a least-squares problem turned into a QP approach with box constraints on the variables. 
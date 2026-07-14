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

## Math and Control Theory

Guppy’s control code runs in parallel with the ROS code on the LattePanda Sigma computer and is managed by a ROS node that communicates with it. The highly optimized closed loop control uses the Eigen and Proxsuite C++ libraries in a layered PID approach. The control structure is responsible for several important tasks, which are outlined in each section below.

### Velocity Control

At its very core, the control code subscribes to the commanded velocity topic (`/cmd_vel`) and the current odometry state topic (`/odometry/filtered`) and attempts to match their 6D vectors of velocities. This is done through feedback controllers on each axis’s velocity error coupled with feedforward terms to combat drag. As velocity in each degree of freedom (three horizontal axes and three rotational) increases, drag in that axis also increases (approximately) quadratically.

Because of this, we use traditional PID controllers to handle most of all velocity control (typically acceleration or deceleration), and a small feedforward term based on the calculated drag to hold a constant velocity. While the code has this capability, we have found in our testing with Guppy that we can rely solely on velocity error PID to maintain consistent velocities, as proportional feedback control gives us acceptable tracks. However, the functionality is there, and we intend to tune the controls even further with more testing time in the future.

### Drag Moments

With many AUVs and ROVs, the center of pressure is not coaxial with the center of mass during movement. This means that movements in any linear degree of freedom can additionally cause unwanted torques. In our experience with Guppy, we have not run across this issue in any noticeable way, due to the relatively slow speeds and its large mass. However, as we implement more tuning and additional mechanisms, this will come into play.

To address these unwanted wrenches, the control code incorporates a “drag effect matrix” which is multiplied by the drag vector to adjust the torques and forces accordingly. The columns of the matrix correspond to the axis that is “causing” the unwanted motion, while the rows correspond to where the motion actually occurs. In an idealized scenario, this would be the 6 × 6 identity matrix. However, if motion forward in the X direction causes an unwanted yaw (due to an uncentered drag force), we can adjust the drag effect matrix to counter it like so:

$$
V_{drag}=
\begin{bmatrix}
1  &  0  &  0  &  0  &  0  &  0  \\
0  &  1  &  0  &  0  &  0  &  0  \\
0  &  0  &  1  &  0  &  0  &  0  \\
0  &  0  &  0  &  1  &  0  &  0  \\
0  &  0  &  0  &  0  &  1  &  0  \\
0.1&  0  &  0  &  0  &  0  &  1
\end{bmatrix}
\begin{bmatrix}
1 \\
0 \\
0 \\
0 \\
0 \\
0
\end{bmatrix}=
\begin{bmatrix}
1 \\
0 \\
0 \\
0 \\
0 \\
0.1
\end{bmatrix}
$$

The above equation shows how the large 6 × 6 matrix can affect the drag feedforward term with an added 0.1 in yaw offset to counteract the drag moment. This can easily scale with multiple axes, allowing for very precise tuning of the submarine’s motion. Note that all code for Guppy follows the standard ROS axis convention of Z-up and X-forward.

### Restoring Forces

There are several restoring forces that act on Guppy at any given time, primarily gravity and buoyancy. Guppy is very positively buoyant while unpowered and must hold itself underwater using thrust. To maintain pseudo-neutral buoyancy, we calculate the difference between the gravity and buoyancy forces in the Z-axis, and then transform that vector to the AUV’s coordinate frame by multiplying it by the inverse quaternion of the sub’s current orientation. We then add this three-dimensional vector to the existing linear feedforward velocity terms. This allows the sub to stay underwater no matter which way it is rotated!	

This correction only accounts for "neutral buoyancy" though and does not offset the restoring torques that the buoyancy wrench imparts on the sub as well. To combat this, we take the rotated "center of buoyancy" offset from the center of mass, and cross-product this vector with the unrotated buoyancy force from before. This essentially computes the sub's tendency to right itself, and the inverse can be applied to the controller’s outputs to negate this.

### Station Keeping

When we aren’t commanding specific velocities, we need the submarine to hold position unless disabled. This is achieved partially via linear and angular velocity PID, because if the setpoint is zero velocity, the sub will resist motion. However, it will not return to its original position if it does move. We have overcome this by switching to a position based PID controller when the commanded velocity is less than some deadband value. The position value is measured from the fused “odometry” value from the DVL and other sensors like the barometer and cameras. The linear PID is very simple, although the angular feedback can get slightly more complicated.

For angular orientation error, we compute the error quaternion $q_{err}=q^{-1}⨂q_{desired}$ where $q$ is the current orientation state of the submarine, and decompose it into its imaginary parts x, y, and z. For small angular error, these error components can be directly inputted into roll, pitch, and yaw feedback controllers. Again, we use a deadband value to determine when to apply each axis’s feedback, although we must carefully apply them in pairs as rotational motion can be tightly coupled across the axes. This quaternion error approximation breaks down for large angular error, but that state should never occur provided that the initial state is the desired one and that the system is correctly tuned.

Not only does this station keeping allow us to hold our pose, but we also expose a `/reset_holding_pose` service that allows other ROS nodes to change the holding state position. This is used by our navigation actions, which decompose trajectories into a series of staggered waypoints, each of which can be applied to the station keeping code in a timed sequence to follow a path. This is exactly what helped us complete our prequalification run in early May.

### Thrust Allocation

After all these individual components assemble into one large six-dimensional AUV wrench vector, we need to convert that into an eight-dimensional motor thrust vector. To do so, we first converted all motor positions into an 6 × 8 matrix of motor coefficients. This is done by taking their x, y, and z locations as well as their spherical angles φ and θ and calculating the cartesian coefficients of each motor. Next, we solve the following least squares problem:

$$A\overset{\rightharpoonup}{x}=s$$

Where A corresponds to the motor coefficient matrix, $\overset{\rightharpoonup}{x}$ to the eight motor thrusts, and s to the desired output wrench. We solve this using least squares, because we need to constrain the motor outputs on their maximum forwards and reversed thrusts. There is also no guaranteed solution under saturation (at very high speeds or some combined movements), which means that simply solving this matrix equation will often fail. That makes this a box-constrained least-squares problem.

To optimize the solving of this and implement it in an existing library (Proxsuite), we have converted this problem into a Quadratic Programming (QP) problem instead. Many least-squares minimization problems can be rewritten as QP problems, as noted by [S. Caron](https://scaron.info/blog/conversion-from-least-squares-to-quadratic-programming.html). Caron describes how to perform this transfer, where we end up with:

$$\min_{\overset{\rightharpoonup}{x}\in\mathbb{R}^8}\frac{1}{2}\overset{\rightharpoonup}{x}^T H\overset{\rightharpoonup}{x}+g^T\overset{\rightharpoonup}{x}$$

given that:

$$A\overset{\rightharpoonup}{x}=s$$

$$\overset{\rightharpoonup}{T}_lower \le \overset{\rightharpoonup}{x} \le \overset{\rightharpoonup}{T}_upper$$

$$H=A^TWA$$

$$g=-A^TWs$$

…where M and s are still the motor coefficient matrix and desired output wrench. W corresponds to an optional weight matrix to weight one axis more than the other in the solution. The $\overset{\rightharpoonup}{T}$ constants refer to the lower and upper thrust bounds of the T200 thrusters (in SI units). In this form, the QP problem is easily applied directly to Proxsuite, and can take advantage of historical loop data to better inform the next guesses. To prevent “stable” states in which opposing thrusters fire at full force (yet still cancelling out), we add a very small addition where $H=A^TWA+0.1I_8$.

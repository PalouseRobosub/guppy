#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <control_toolbox/control_toolbox/pid.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "guppy_nav/trajectory.hpp"

#include "guppy_msgs/action/navigate.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#define THRESHOLD 0.1

#define TOTAL_TIME 10 // in seconds
#define ATTACK 0.25 // from 0.0 - 1.0, ATTACK + DECAY must <= 1.0
#define DECAY 0.25

#define TARGET_RATE_MS 10

class NavigateActionServer : public rclcpp::Node {
public:
    explicit NavigateActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("navigate_action_server", options) {
        auto handleGoal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const guppy_msgs::action::Navigate::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "goal request with pose and relative set to %d", goal->relative); //goal->pose

            (void)uuid; // silence unused parameter warning

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handleCancel = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
            RCLCPP_INFO(this->get_logger(), "request to cancel goal");

            (void)goalHandle; // silence unused parameter warning

            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handleAccepted = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
            execute(goalHandle);
        };
        
        _actionServer = rclcpp_action::create_server<guppy_msgs::action::Navigate>(
            this,
            "/navigate",
            handleGoal,
            handleCancel,
            handleAccepted
        );

        _commandVelocityPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel/nav", 10);
        _odomSubscription = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",10,std::bind(&NavigateActionServer::odometryCallback, this, std::placeholders::_1));

        _xPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
        _yPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
        _zPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
        _yawPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
        _pitchPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
        _rollPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
    }

    void odometryCallback(nav_msgs::msg::Odometry::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(_kinematiStateMutex);
        _kinematicState.pose = msg->pose.pose;
        _kinematicState.twist = msg->twist.twist;
    }
private:
    rclcpp_action::Server<guppy_msgs::action::Navigate>::SharedPtr _actionServer;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr _commandVelocityPublisher;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _odomSubscription;

    struct KinematicState {
        geometry_msgs::msg::Pose pose;
        geometry_msgs::msg::Twist twist;
    };

    KinematicState _kinematicState;
    std::mutex _kinematiStateMutex;

    control_toolbox::Pid _xPid, _yPid, _zPid, _yawPid, _pitchPid, _rollPid;

    struct Trajectory3 {
        Trajectory x, y, z;
        
        explicit Trajectory3(const Eigen::Vector3d& startVelocity, const Eigen::Vector3d& endVelocity, double attack, double decay, double totalTime, const Eigen::Vector3d& targetPosition) :
        x(Trajectory(startVelocity.x(), endVelocity.x(), attack, decay, totalTime, targetPosition.x())),
        y(Trajectory(startVelocity.y(), endVelocity.y(), attack, decay, totalTime, targetPosition.y())),
        z(Trajectory(startVelocity.z(), endVelocity.z(), attack, decay, totalTime, targetPosition.z())) {}

        Eigen::Vector3d velocity(double elapsed) const {
            return Eigen::Vector3d(x.getTargetVelocity(elapsed), y.getTargetVelocity(elapsed), z.getTargetVelocity(elapsed));
        }

        Eigen::Vector3d position(double elapsed) const {
            return Eigen::Vector3d(x.getTargetPosition(elapsed), y.getTargetPosition(elapsed), z.getTargetPosition(elapsed));
        }
    };

    struct OrientationSolver {
        Eigen::Quaterniond initialOrientation, finalOrientation;
        double totalTime;

        explicit OrientationSolver(const Eigen::Quaterniond& initialOrientation, const Eigen::Quaterniond& finalOrientation, double totalTime) :
        initialOrientation(initialOrientation), finalOrientation(finalOrientation), totalTime(totalTime) {}

        Eigen::Vector3d error(double elapsed, const Eigen::Quaterniond& currentOrientation) const {
            double alpha = std::clamp(elapsed / totalTime, 0.0, 1.0);

            Eigen::Quaterniond targetOrientation = initialOrientation.slerp(alpha, finalOrientation);

            Eigen::Quaterniond orientationError = currentOrientation.inverse() * targetOrientation;
            Eigen::AngleAxisd angleAxisError(orientationError);

            return angleAxisError.axis() * angleAxisError.angle();
        }
    };

    inline static const control_toolbox::AntiWindupStrategy antiWindupStrategy = []{
        control_toolbox::AntiWindupStrategy strategy;
        strategy.set_type("back_calculation");
        return strategy;
    }();

    KinematicState getKinematicState() {
        std::lock_guard<std::mutex> lock(_kinematiStateMutex);
        return _kinematicState;
    }

    geometry_msgs::msg::Twist computeCommandVelocity(const Trajectory3& trajectory, const OrientationSolver& orientationSolver, double elapsed, double delta, const KinematicState& state, const Eigen::Vector3d& initialPosition) {
        const auto targetPositionWorld = trajectory.position(elapsed); // world
        const auto targetVelocityWorld = trajectory.velocity(elapsed); // world

        const Eigen::Vector3d currentPosition(state.pose.position.x, state.pose.position.y, state.pose.position.z); // world!
        const auto relativePosition = currentPosition - initialPosition; // still world
        const auto worldError = targetPositionWorld - relativePosition;

        Eigen::Quaterniond currentOrientation(state.pose.orientation.w, state.pose.orientation.x, state.pose.orientation.y, state.pose.orientation.z); // world!
        currentOrientation.normalize();

        const auto localError = currentOrientation.inverse() * worldError;
        const auto targetVelocityLocal = currentOrientation.inverse() * targetVelocityWorld;
        const auto angularErrorLocal = orientationSolver.error(elapsed, currentOrientation);

        geometry_msgs::msg::Twist commandVelocity;

        commandVelocity.angular.z = _yawPid.compute_command(angularErrorLocal.z(), rclcpp::Duration::from_seconds(delta));
        commandVelocity.angular.y = _pitchPid.compute_command(angularErrorLocal.y(), rclcpp::Duration::from_seconds(delta));
        commandVelocity.angular.x = _rollPid.compute_command(angularErrorLocal.x(), rclcpp::Duration::from_seconds(delta));

        commandVelocity.linear.x = targetVelocityLocal.x() + _xPid.compute_command(localError.x(), rclcpp::Duration::from_seconds(delta));
        commandVelocity.linear.y = targetVelocityLocal.y() + _yPid.compute_command(localError.y(), rclcpp::Duration::from_seconds(delta));
        commandVelocity.linear.z = targetVelocityLocal.z() + _zPid.compute_command(localError.z(), rclcpp::Duration::from_seconds(delta));

        return commandVelocity;
    };

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
        const auto& goal = goalHandle->get_goal();

        const auto initialState = getKinematicState();

        Eigen::Vector3d initialPosition(initialState.pose.position.x, initialState.pose.position.y, initialState.pose.position.z); //world
        Eigen::Quaterniond initialOrientation(initialState.pose.orientation.w, initialState.pose.orientation.x, initialState.pose.orientation.y, initialState.pose.orientation.z); //world
        initialOrientation.normalize();

        Eigen::Vector3d relativeFinalPosition; // still world, just makes origin (0, 0, 0) (may have to be converted if user submitted local coordinates)
        
        const auto& finalPose = goal->pose;
        Eigen::Quaterniond finalOrientation(finalPose.orientation.w, finalPose.orientation.x, finalPose.orientation.y, finalPose.orientation.z); // will end up in world
        finalOrientation.normalize();

        if (goal->relative) {
            relativeFinalPosition << finalPose.position.x, finalPose.position.y, finalPose.position.z;
            finalOrientation = initialOrientation * finalOrientation;
        } else relativeFinalPosition << finalPose.position.x - initialPosition.x(), finalPose.position.y - initialPosition.y(), finalPose.position.z - initialPosition.z();

        Eigen::Vector3d startVelocity(initialState.twist.linear.x, initialState.twist.linear.y, initialState.twist.linear.z); // world

        Trajectory3 trajectory(startVelocity, Eigen::Vector3d::Zero(), ATTACK, DECAY, TOTAL_TIME, relativeFinalPosition);

        OrientationSolver orientationSolver(initialOrientation, finalOrientation, TOTAL_TIME);

        rclcpp::Rate rate(1000.0 / TARGET_RATE_MS);

        _xPid.reset();
        _yPid.reset();
        _zPid.reset();
        _yawPid.reset();
        _pitchPid.reset();
        _rollPid.reset();

        auto clock = this->get_clock();
        rclcpp::Time start = clock->now();
        rclcpp::Time last = start;
        
        auto feedback = std::make_shared<guppy_msgs::action::Navigate::Feedback>();
        auto result = std::make_shared<guppy_msgs::action::Navigate::Result>();

        while (rclcpp::ok()) {
            if (goalHandle->is_canceling()) {
                result->target_reached = false;
                goalHandle->canceled(result);

                geometry_msgs::msg::Twist zeroTwist; // publish zero twist
                _commandVelocityPublisher->publish(zeroTwist);

                return;
            }

            auto now = clock->now();
            double elapsed = (now - start).seconds();
            double delta = std::clamp((now - last).seconds(), 1e-4, 0.05);
            last = now;

            auto state = getKinematicState();
            auto commandVelocity = computeCommandVelocity(trajectory, orientationSolver, elapsed, delta, state, initialPosition);

            Eigen::Vector3d currentPosition;
            currentPosition << state.pose.position.x, state.pose.position.y, state.pose.position.z;
            auto relativeCurrentPosition = currentPosition - initialPosition;

            feedback->progress.position.x = currentPosition.x(), feedback->progress.position.x = currentPosition.y(), feedback->progress.position.x = currentPosition.z();

            _commandVelocityPublisher->publish(commandVelocity);
            goalHandle->publish_feedback(feedback);

            if (elapsed >= TOTAL_TIME) break;

            rate.sleep();
        }

        geometry_msgs::msg::Twist zeroTwist; // publish zero twist
        _commandVelocityPublisher->publish(zeroTwist);

        if (rclcpp::ok()) {
            result->pose = getKinematicState().pose;
            result->target_reached = true; // shouldn't assume true, fix that by calcing the hypotenuse

            goalHandle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    }
};

RCLCPP_COMPONENTS_REGISTER_NODE(NavigateActionServer)

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<NavigateActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);

    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}
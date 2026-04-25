#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <control_toolbox/control_toolbox/pid.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
            "navigate",
            handleGoal,
            handleCancel,
            handleAccepted
        );

        _commandVelocityPublisher = create_publisher<geometry_msgs::msg::Twist>("cmd_vel/nav", 10);
        _odomSubscription = create_subscription<nav_msgs::msg::Odometry>("/odometry/filtered",10,std::bind(&NavigateActionServer::odometryCallback, this, std::placeholders::_1));

        _xPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy); // this broke please help
        _yPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
        _zPid.set_gains(1.5, 0.0, 0.1, std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(), antiWindupStrategy);
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

    control_toolbox::Pid _xPid, _yPid, _zPid;

    struct Vector3 {
        double x{0}, y{0}, z{0};

        Vector3() = default;
        Vector3(double x, double y, double z) : x(x), y(y), z(z) {}
    };

    struct Trajectory3 {
        Trajectory x, y, z;
        
        explicit Trajectory3(const Vector3& startVelocity, const Vector3& endVelocity, double attack, double decay, double totalTime, const Vector3& targetPosition) :
        x(Trajectory(startVelocity.x, endVelocity.x, attack, decay, totalTime, targetPosition.x)),
        y(Trajectory(startVelocity.y, endVelocity.y, attack, decay, totalTime, targetPosition.y)),
        z(Trajectory(startVelocity.z, endVelocity.z, attack, decay, totalTime, targetPosition.z)) {}

        Vector3 velocity(double time) const {
            return {x.getTargetVelocity(time), y.getTargetVelocity(time), z.getTargetVelocity(time)};
        }

        Vector3 position(double time) const {
            return {x.getTargetPosition(time), y.getTargetPosition(time), z.getTargetPosition(time)};
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

    Vector3 getRPY(const geometry_msgs::msg::Quaternion& msg) {
        tf2::Quaternion quaternion;
        tf2::fromMsg(msg, quaternion);
        tf2::Matrix3x3 matrix(quaternion);

        Vector3 orientation;
        matrix.getRPY(orientation.x, orientation.y, orientation.z); // (x) roll, (y) pitch, (z) yaw
        return orientation;
    }


    Vector3 transformWorldToLocal(const Vector3& world, const Vector3& rpy) {
        Vector3 local;

        local.x =  cos(rpy.z) * world.x + sin(rpy.z) * world.y;
        local.y = -sin(rpy.z) * world.x + cos(rpy.z) * world.y;

        local.z =  cos(rpy.y) * world.z - sin(rpy.y) * local.x;

        return local;
    } 

    geometry_msgs::msg::Twist computeCommandVelocity(const Trajectory3& trajectory, double time, double dt, const KinematicState& state, const Vector3& initialPosition) {
        auto targetPosition = trajectory.position(time);
        auto targetVelocity = trajectory.velocity(time);

        Vector3 relativePosition{state.pose.position.x - initialPosition.x, state.pose.position.y - initialPosition.y, state.pose.position.z - initialPosition.z};
        Vector3 worldError{targetPosition.x - relativePosition.x, targetPosition.y - relativePosition.y, targetPosition.z - relativePosition.z}; // need to translate the state to relative coordinates, i.e. from initial position

        auto rpy = getRPY(state.pose.orientation);
        auto localError = transformWorldToLocal(worldError, rpy);

        geometry_msgs::msg::Twist commandVelocity;

        commandVelocity.linear.x = targetVelocity.x + _xPid.compute_command(localError.x, rclcpp::Duration::from_seconds(dt));
        commandVelocity.linear.y = targetVelocity.y + _yPid.compute_command(localError.y, rclcpp::Duration::from_seconds(dt));
        commandVelocity.linear.z = targetVelocity.z + _zPid.compute_command(localError.z, rclcpp::Duration::from_seconds(dt));

        return commandVelocity;
    };

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goalHandle) {
        const auto& goal = goalHandle->get_goal();

        auto initialState = getKinematicState();

        Vector3 initialPosition{initialState.pose.position.x, initialState.pose.position.y, initialState.pose.position.z};
        auto finalPose = goal->pose; // not relative
        Vector3 relativeFinalPosition = goal->relative ? Vector3{finalPose.position.x, finalPose.position.y, finalPose.position.z} : Vector3{finalPose.position.x - initialPosition.x, finalPose.position.y - initialPosition.y, finalPose.position.z - initialPosition.z};

        auto initialLinearVelocity = initialState.twist.linear;
        Vector3 startVelocity{initialLinearVelocity.x, initialLinearVelocity.y, initialLinearVelocity.z};

        Trajectory3 trajectory(startVelocity, {0, 0, 0}, ATTACK, DECAY, TOTAL_TIME, relativeFinalPosition);

        rclcpp::Rate rate(1000.0 / TARGET_RATE_MS);

        _xPid.reset();
        _yPid.reset();
        _zPid.reset();

        auto clock = this->get_clock();
        rclcpp::Time start = clock->now();
        rclcpp::Time last = start;
        
        auto feedback = std::make_shared<guppy_msgs::action::Navigate::Feedback>();
        auto result = std::make_shared<guppy_msgs::action::Navigate::Result>();

        while (rclcpp::ok()) {
            if (goalHandle->is_canceling()) {
                result->target_reached = false;
                goalHandle->canceled(result);

                return;
            }

            auto now = clock->now();
            double elapsed = (now - start).seconds();
            double delta = std::clamp((now - last).seconds(), 1e-4, 0.05);
            last = now;

            auto state = getKinematicState();
            auto commandVelocity = computeCommandVelocity(trajectory, elapsed, delta, state, initialPosition);

            feedback->progress = state.pose;

            _commandVelocityPublisher->publish(commandVelocity);
            goalHandle->publish_feedback(feedback);

            if (elapsed >= TOTAL_TIME) break;

            rate.sleep();
        }

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
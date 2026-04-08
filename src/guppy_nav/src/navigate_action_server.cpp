#include <functional>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "guppy_msgs/action/navigate.hpp"
#include "geometry_msgs/msg/twist.hpp"

#define THRESHOLD 0.1

class NavigateActionServer : public rclcpp::Node {
public:
    explicit NavigateActionServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) : Node("navigate_action_server", options) {
        auto handle_goal = [this](const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const guppy_msgs::action::Navigate::Goal> goal) {
            RCLCPP_INFO(this->get_logger(), "goal request with pose and relative set to %d", goal->relative); //goal->pose

            (void)uuid; // silence unused parameter warning

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goal_handle) {
            RCLCPP_INFO(this->get_logger(), "request to cancel goal");

            (void)goal_handle; // silence unused parameter warning

            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted = [this](const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goal_handle) {
            // to avoid blocking calling thread, return lambda quickly and execute on seperate thread
            auto execute_in_thread = [this, goal_handle](){
                return this->execute(goal_handle);
            };

            std::thread{execute_in_thread}.detach();
        };

        this->action_server_ = rclcpp_action::create_server<guppy_msgs::action::Navigate>(
            this,
            "navigate",
            handle_goal,
            handle_cancel,
            handle_accepted
        );
    }
private:
    rclcpp_action::Server<guppy_msgs::action::Navigate>::SharedPtr action_server_;

    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<guppy_msgs::action::Navigate>> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "executing goal");

        rclcpp::Rate loop_rate(1); //?

        const auto goal = goal_handle->get_goal();

        auto feedback = std::make_shared<guppy_msgs::action::Navigate::Feedback>();

        auto result = std::make_shared<guppy_msgs::action::Navigate::Result>();

        for(int i = 1; /*(i < goal->order) &&*/ rclcpp::ok(); ++i) { //what is this?
            if (goal_handle->is_canceling()) {
                //result->sequence = sequenece;
                goal_handle->canceled(result);

                RCLCPP_INFO(this->get_logger(), "goal cancelled");

                return;
            }

            goal_handle->publish_feedback(feedback);
            
            RCLCPP_INFO(this->get_logger(), "published feeback");

            loop_rate.sleep();
        }

        if (rclcpp::ok()) {
            // result->pose = ...
            // result->

            goal_handle->succeed(result);

            RCLCPP_INFO(this->get_logger(), "goal succeeded");
        }
    };
};

RCLCPP_COMPONENTS_REGISTER_NODE(NavigateActionServer)
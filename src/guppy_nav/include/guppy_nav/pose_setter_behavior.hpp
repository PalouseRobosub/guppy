#pragma once

#include "guppy_msgs/action/navigate.hpp"

#include <behaviortree_ros2/bt_action_node.hpp>

class NavigateBehavior: public BT::RosActionNode<guppy_msgs::action::Navigate> {
public:
    NavigateBehavior(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) : BT::RosActionNode<guppy_msgs::action::Navigate>(name, conf, params) {}

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<double>("x"), BT::InputPort<double>("y"), BT::InputPort<double>("z"),
            BT::InputPort<double>("qw"), BT::InputPort<double>("qx"), BT::InputPort<double>("qy"), BT::InputPort<double>("qz"),
            BT::InputPort<bool>("local"), BT::InputPort<double>("timeout"),
            BT::InputPort<bool>("continueOnTimeout")
        });
    }

    bool setGoal(BT::RosActionNode<guppy_msgs::action::Navigate>::Goal& goal) override {
        getInput("x", goal.pose.position.x);
        getInput("y", goal.pose.position.y);
        getInput("z", goal.pose.position.z);
        getInput("qw", goal.pose.orientation.w);
        getInput("qx", goal.pose.orientation.x);
        getInput("qy", goal.pose.orientation.y);
        getInput("qz", goal.pose.orientation.z);
        getInput("local", goal.local);
        getInput("timeout", goal.timeout);
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wrapped) override {
        // should do?
        return BT::NodeStatus::SUCCESS;
    }

    virtual BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
        RCLCPP_ERROR(logger(), "pose setter treenode error... %s", BT::toStr(error));
        bool continueOnTimeout = false;
        getInput("continueOnTimeout", continueOnTimeout);
        if (continueOnTimeout) return BT::NodeStatus::SUCCESS;
        else return BT::NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
        return BT::NodeStatus::RUNNING;
    }
};

#include "guppy_msgs/action/navigate.hpp"

#include <behaviortree_ros2/bt_action_node.hpp>

class NavigateAction: public BT::RosActionNode<guppy_msgs::action::Navigate> {
public:
    NavigateAction(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) : BT::RosActionNode<guppy_msgs::action::Navigate>(name, conf, params) {}

    static BT::PortList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<double>("x"), BT::InputPort<float>("y"), BT::InputPort<float>("z"),
            BT::InputPort<double>("qw"), BT::InputPort<double>("qw"), BT::InputPort<double>("qw"), BT::InputPort<double>("qw"),
            BT::InputPort<bool>("local"), BT::InputPort<double>("timeout")
        });
    }

    bool setGoal(BT::RosActionNode::Goal& goal) override {
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

    BT::NodeStatus onResultReceived(const BT::WrappedResult& wrapped) override {
        // should do?
        return BT::NodeStatus::SUCCESS;
    }

    virtual NodeStatus onFailure(BT::ActionNodeErrorCode error) override {
        RCLCPP_ERROR(logger(), "error: %d", error);
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onFeedback(const std::shared_ptr<const BT::Feedback> feedback) {
        return BT::NodeStatus::RUNNING;
    }
};

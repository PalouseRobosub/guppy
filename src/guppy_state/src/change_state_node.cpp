#include <behaviortree_ros2/bt_service_node.hpp>
#include <guppy_msgs/srv/change_state.hpp>

class ChangeStateNode : public BT::RosServiceNode<guppy_msgs::srv::ChangeState>
{
public:
    ChangeStateNode(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params)
    : RosServiceNode<guppy_msgs::srv::ChangeState>(name, conf, params)
    {}

    static BT::PortsList providedPorts()
    {
        return providedBasicPorts({
            BT::InputPort<float>("state")
        });
    }

    bool setRequest(Request::SharedPtr& request) override
    {
        getInput("state", request->new_state);

        return true;
    }

    BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override
    {
        RCLCPP_DEBUG(logger(), "ChangeStateNode success");
        return BT::NodeStatus::SUCCESS;
    }
};
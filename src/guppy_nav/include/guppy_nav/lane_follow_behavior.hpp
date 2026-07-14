
#pragma once
 
#include <behaviortree_ros2/bt_action_node.hpp>
#include "guppy_msgs/action/lane_follow.hpp"

class LaneFollowBehavior : public BT::RosActionNode<guppy_msgs::action::LaneFollow>{
    public:
        LaneFollowBehavior(const std::string& name, const BT::NodeConfig& conf, const BT::RosNodeParams& params) : BT::RosActionNode<guppy_msgs::action::LaneFollow>(name,conf,params){}

    static BT::PortsList providedPorts(){
        return providedBasicPorts({
            BT::InputPort<double>("lost_timeout",2.0,"seconds without a lane sighted befoe the action fails")

        });
    }

    bool setGoal(RosActionNode::Goal& goal)override{
        getInput("lost_timeout",goal.lost_timeout);
        return true;
    }

    BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
        if (wr.result->success){
            RCLCPP_INFO(logger(),"LaneFolow succeeded : %s",wr.result->message.c_str());
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::FAILURE;
    }
    BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) {
        (void)feedback;
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override{
        RCLCPP_ERROR(logger(), "LaneFollow action error: %d",static_cast<int>(error));
        return BT::NodeStatus::FAILURE;
    }
};
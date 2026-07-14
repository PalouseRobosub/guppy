#include <algorithm>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/basic_types.h>
#include <behaviortree_cpp/tree_node.h>

#include <behaviortree_ros2/bt_topic_sub_node.hpp>
#include "guppy_msgs/msg/corner_detection.hpp"
#include "guppy_msgs/msg/corner_detection_list.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <guppy_msgs/msg/corner_detection.hpp>
#include <vision_msgs/msg/point2_d.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace BT {

template <>
inline guppy_msgs::msg::CornerDetection convertFromString(StringView str) {
    auto fields = splitString(str, '|');

    guppy_msgs::msg::CornerDetection msg;

    msg.name = std::string(fields[0]);
    msg.confidence = convertFromString<float>(fields[1]);
    msg.square = convertFromString<bool>(fields[2]);

    if (!fields[3].empty()) {
        auto corner_strings = splitString(fields[3], ';');

        for (const auto& c : corner_strings) {
            auto xy = splitString(c, ',');

            if (xy.size() != 2) throw RuntimeError("Invalid Point2D");

            vision_msgs::msg::Point2D pt;
            pt.x = convertFromString<double>(xy[0]);
            pt.y = convertFromString<double>(xy[1]);

            msg.corners.push_back(pt);
        }
    }

    if (!fields[4].empty()) {
        auto point_strings = splitString(fields[4], ';');

        for (const auto& p : point_strings) {
            auto xyz = splitString(p, ',');

            geometry_msgs::msg::Point pt;
            pt.x = convertFromString<double>(xyz[0]);
            pt.y = convertFromString<double>(xyz[1]);
            pt.z = convertFromString<double>(xyz[2]);

            msg.dimension_points.push_back(pt);
        }
    }

    return msg;
}

template <>
inline std::string toStr<guppy_msgs::msg::CornerDetection>(const guppy_msgs::msg::CornerDetection& msg) {
    std::ostringstream out;

    out << msg.name << '|'
        << msg.confidence << '|'
        << (msg.square ? "true" : "false") << '|';

    // corners
    for (size_t i = 0; i < msg.corners.size(); i++) {
        const auto& p = msg.corners[i];
        out << p.x << "," << p.y;

        if (i + 1 < msg.corners.size()) out << ";";
    }

    out << '|';

    // dimension_points
    for (size_t i = 0; i < msg.dimension_points.size(); i++) {
        const auto& p = msg.dimension_points[i];
        out << p.x << "," << p.y << "," << p.z;

        if (i + 1 < msg.dimension_points.size()) out << ";";
    }

    return out.str();
}

} // namespace BT

class AcquireDetection : public BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList> {
public:
    AcquireDetection(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params)
    : BT::RosTopicSubNode<guppy_msgs::msg::CornerDetectionList>(name, config, params) { }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<std::string>("target", "any"),
            BT::OutputPort<guppy_msgs::msg::CornerDetection>("detection")
        });
    }

    BT::NodeStatus onTick(const std::shared_ptr<guppy_msgs::msg::CornerDetectionList>& msg) override {
        if (!msg) return BT::NodeStatus::FAILURE; // no detections in list

        auto detections = msg->detections;

        std::string target;
        getInput("target", target);

        std::sort(detections.begin(), detections.end(),
            [](guppy_msgs::msg::CornerDetection first, guppy_msgs::msg::CornerDetection second) {
                return first.confidence > second.confidence;
            }
        ); // sort by confidence first

        if (target == "any") setOutput("detection", detections.front());
        else {
            auto it = std::find_if(detections.begin(), detections.end(), [target](const guppy_msgs::msg::CornerDetection detection) { return matchesTarget(detection, target); });
            if (it == detections.end()) return BT::NodeStatus::FAILURE; // no detection matching target

            setOutput("detection", *it);
        }

        return BT::NodeStatus::SUCCESS;
    }
private:
    static bool matchesTarget(const guppy_msgs::msg::CornerDetection detection, const std::string& target) {
        return detection.name == target;
    }
};

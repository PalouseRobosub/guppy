#include "guppy_control/can_actuator_interface.hpp"

namespace can_actuator_interface {

hardware_interface::CallbackReturn CANActuatorInterface::on_init(const hardware_interface::HardwareInfo &info) {
    CallbackReturn result = hardware_interface::ActuatorInterface::on_init(info);
    if (result != CallbackReturn::SUCCESS) return result;

    interface_ = info_.hardware_parameters.at("can_interface");

    for (auto joint: info_.joints) {
        can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CANActuatorInterface::on_configure(const rclcpp_lifecycle::State &previous_state) {
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
        RCLCPP_ERROR(this->get_logger(), "failed to open CAN socket");
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct ifreq ifr;
    std::strcpy(ifr.ifr_name, interface_.c_str());
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
        RCLCPP_ERROR(this->get_logger(), "SIOCGIFINDEX failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
        RCLCPP_ERROR(this->get_logger(), "CAN bind failed");
        return hardware_interface::CallbackReturn::ERROR;
    }

    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn CANActuatorInterface::on_cleanup(const rclcpp_lifecycle::State &previous_state) {
    close(sock_);
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CANActuatorInterface::read(const rclcpp::Time & time, const rclcpp::Duration & period) {
    RCLCPP_DEBUG_ONCE(this->get_logger(), "write() called: time: %ld, duration: %ld", time.nanoseconds(), period.nanoseconds());
    return hardware_interface::return_type::OK;
}


}
#include "guppy_control/t200_interface.hpp"

namespace t200_interface {

hardware_interface::CallbackReturn T200Interface::on_shutdown(const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_INFO(this->get_logger(), "shutting down all t200 actuators: stopping motors (from state %s)", previous_state.label().c_str());

    if (!send_over_can_all(0.0)) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn T200Interface::on_activate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(this->get_logger(), "Activating all t200 actuators with zero speed (from state %s)", previous_state.label().c_str());

    if (!send_over_can_all(0.0)) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn T200Interface::on_deactivate(const rclcpp_lifecycle::State &previous_state) {
    RCLCPP_INFO(this->get_logger(), "deactivating all t200 actuators: stopping motors (from state %s)", previous_state.label().c_str());

    if (!send_over_can_all(0.0)) {
        return hardware_interface::CallbackReturn::ERROR;
    }
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn T200Interface::on_error(const rclcpp_lifecycle::State& previous_state) {
    RCLCPP_ERROR(this->get_logger(), "error from previour state %s", previous_state.label().c_str());
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> T200Interface::export_state_interfaces() {
    // none
    return std::vector<hardware_interface::StateInterface>();
}

std::vector<hardware_interface::CommandInterface> T200Interface::export_command_interfaces() {
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    
    for (size_t i = 0; i < info_.joints.size(); i++) {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, 
            &velocity_commanded_[i]
        ));
    }

    return command_interfaces;
}

hardware_interface::return_type T200Interface::write(const rclcpp::Time& time, const rclcpp::Duration& period) {
    RCLCPP_DEBUG_ONCE(this->get_logger(), "write() called: time: %ld, duration: %ld", time.nanoseconds(), period.nanoseconds());

    for (size_t i = 0; i < info_.joints.size(); i++) {
        int id = can_ids_.at(i);
        double v = velocity_commanded_.at(i);
        if (!send_over_can(id, v)) {
            RCLCPP_ERROR(this->get_logger(), "failed to send can frame %.2f to id %x", v, (unsigned int) id);
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}
}  // namespace t200_interface

PLUGINLIB_EXPORT_CLASS(t200_interface::T200Interface, hardware_interface::ActuatorInterface)

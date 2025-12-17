#ifndef GUPPY_T200_INTERFACE_H
#define GUPPY_T200_INTERFACE_H

#include "can_actuator_interface.hpp"
namespace t200_interface
{
class T200Interface : public can_actuator_interface::CANActuatorInterface {
public:
    T200Interface() {};
    ~T200Interface() {};

    hardware_interface::CallbackReturn on_shutdown (const rclcpp_lifecycle::State &previous_state);
    hardware_interface::CallbackReturn on_activate (const rclcpp_lifecycle::State &previous_state);
    hardware_interface::CallbackReturn on_deactivate (const rclcpp_lifecycle::State &previous_state);
    hardware_interface::CallbackReturn on_error (const rclcpp_lifecycle::State &previous_state);
 
    std::vector<hardware_interface::StateInterface> export_state_interfaces();
    std::vector<hardware_interface::CommandInterface> export_command_interfaces();

    hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period);

private:
    std::vector<double> velocity_commanded_;
};
}

#endif
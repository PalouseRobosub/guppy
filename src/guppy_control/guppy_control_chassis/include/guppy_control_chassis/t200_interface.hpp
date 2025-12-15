#include "hardware_interface/actuator_interface.hpp"

 class T200Thruster : public hardware_interface::ActuatorInterface {
    public:
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams & params) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;
 }
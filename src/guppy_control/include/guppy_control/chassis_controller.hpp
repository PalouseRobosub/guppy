// #ifndef GUPPY_CHASSIS_HPP_
// #define GUPPY_CHASSIS_HPP_

// #include "controller_interface/controller_interface.hpp"
// #include "rclcpp_lifecycle/state.hpp"


// namespace GUPPY_CHASSIS
// {

// using CmdType = control_msgs::msg::DynamicInterfaceGroupValues;
// using StateType = control_msgs::msg::DynamicInterfaceGroupValues;
// using CallbackReturn = controller_interface::CallbackReturn;
// using InterfacesNames = std::vector<std::string>;

// class ControllerName : public controller_interface::ControllerInterface
// {
// public:
//   GpioCommandController();

//   controller_interface::InterfaceConfiguration command_interface_configuration() const override;

//   controller_interface::InterfaceConfiguration state_interface_configuration() const override;

//   CallbackReturn on_init() override;

//   CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

//   CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

//   CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

//   controller_interface::return_type update(
//     const rclcpp::Time & time, const rclcpp::Duration & period) override;};

// }  // namespace GUPPY_CHASSIS

// #endif  // GUPPY_CHASSIS_HPP_
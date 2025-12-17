#ifndef CAN_ACTUATOR_INTERFACE_H
#define CAN_ACTUATOR_INTERFACE_H

#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/actuator_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>

namespace can_actuator_interface
{
class CANActuatorInterface : public hardware_interface::ActuatorInterface {
public:
    CANActuatorInterface();
    ~CANActuatorInterface();

    hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info);
    hardware_interface::CallbackReturn on_configure (const rclcpp_lifecycle::State &previous_state);
    hardware_interface::CallbackReturn on_cleanup (const rclcpp_lifecycle::State &previous_state);

    virtual hardware_interface::CallbackReturn on_shutdown (const rclcpp_lifecycle::State &previous_state) = 0;
    virtual hardware_interface::CallbackReturn on_activate (const rclcpp_lifecycle::State &previous_state) = 0;
    virtual hardware_interface::CallbackReturn on_deactivate (const rclcpp_lifecycle::State &previous_state) = 0;
    virtual hardware_interface::CallbackReturn on_error (const rclcpp_lifecycle::State &previous_state) = 0;
 
    virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() = 0;
    virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() = 0;

    virtual hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;
    virtual hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

    // template definitions go in the header file
    template <typename T>
    bool send_over_can(int id, T value) {
        if (sizeof(T) > 8) {
            RCLCPP_ERROR(this->get_logger(), "type length (%d) is too large for CAN", sizeof(T));
        }

        struct can_frame frame;
        frame.can_id = id;
        frame.can_dlc = sizeof(T);

        std::memcpy(frame.data, &value, sizeof(T));

        if (::write(sock_, &frame, sizeof(struct can_frame)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "CAN write failed: %s", strerror(errno));
            return false;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "Published throttle %.3f to CAN ID %x", value, (unsigned int) id);
            return true;
        }
    }

    template <typename T>
    bool send_over_can_all(T value) {
        bool okay = true;
        for (int id: can_ids_) {
            okay = okay & send_over_can(id, value);
        }
        return okay;
    }

protected:
    int sock_;

    std::vector<int> can_ids_;
    std::string interface_;
};
}

#endif
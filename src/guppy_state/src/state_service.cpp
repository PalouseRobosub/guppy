#include <chrono>
#include <memory>
#include <cstdint>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "guppy_msgs/srv/change_state.hpp"                                                                                              
#include "std_msgs/msg/u_int8.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class StateManager : public rclcpp::Node {
    public:
        StateManager() : Node("state_manager"), current_state_(guppy_msgs::srv::ChangeState::Request::INITIAL) {
            state_publisher_ = this->create_publisher<std_msgs::msg::UInt8>("state", 10); // what should the queue size be?
            state_service_ = this->create_service<guppy_msgs::srv::ChangeState>(
                "change_state",
                std::bind(&StateManager::transition_callback, this, std::placeholders::_1, std::placeholders::_2)
            );
            timer_ = this->create_wall_timer(50ms, std::bind(&StateManager::on_timer, this));

            auto holding_callback   = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->holding_twist_ = *msg; };
            auto nav_callback       = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->nav_twist_ = *msg; };
            auto task_callback      = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->task_twist_ = *msg; };
            auto teleop_callback    = [this](geometry_msgs::msg::Twist::UniquePtr msg) -> void { this->teleop_twist_ = *msg; };

            this->holding_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel/holding", 10, holding_callback);
            this->nav_subscription_     = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel/nav", 10, nav_callback);
            this->task_subscription_    = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel/task", 10, task_callback);
            this->teleop_subscription_  = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel/teleop", 10, teleop_callback);

            cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10); // this one too
        }
    
    private:
        bool is_valid_state(uint8_t state) {
            switch (state) {
                case guppy_msgs::srv::ChangeState::Request::INITIAL:
                case guppy_msgs::srv::ChangeState::Request::HOLDING:
                case guppy_msgs::srv::ChangeState::Request::NAV:
                case guppy_msgs::srv::ChangeState::Request::TASK:
                case guppy_msgs::srv::ChangeState::Request::TELEOP:
                case guppy_msgs::srv::ChangeState::Request::DISABLED:
                case guppy_msgs::srv::ChangeState::Request::FAULT:
                    return true;
                default:
                    return false;
            }
        }

        void transition_callback(
            const std::shared_ptr<guppy_msgs::srv::ChangeState::Request> request,
            std::shared_ptr<guppy_msgs::srv::ChangeState::Response> response
        ) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State transition request...");   
            
            if (!is_valid_state(request->state)) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Invalid state passed in transition sercice!");
                response->success = false;
                return;
            }

            // TODO switch logic should be handeled here NOT in StateManager#publishState()
            
            response->success = this->publish_state(request->state);                                                                                 
        }

        bool publish_state(uint8_t state) {
            auto message = std_msgs::msg::UInt8();
            message.data = state;
            this->state_publisher_->publish(message);
            this->current_state_ = state;
            return true;                                                                                  
        }

        void on_timer() {
            switch (this->current_state_) {
                case guppy_msgs::srv::ChangeState::Request::INITIAL:    this->handle_initial();     break;
                case guppy_msgs::srv::ChangeState::Request::HOLDING:    this->handle_holding();     break;
                case guppy_msgs::srv::ChangeState::Request::NAV:        this->handle_nav();         break;
                case guppy_msgs::srv::ChangeState::Request::TASK:       this->handle_task();        break;
                case guppy_msgs::srv::ChangeState::Request::TELEOP:     this->handle_teleop();      break;
                case guppy_msgs::srv::ChangeState::Request::DISABLED:   this->handle_disabled();    break;
                case guppy_msgs::srv::ChangeState::Request::FAULT:      this->handle_fault();       break;
            }
        }

        // state handlers
        void handle_initial() {
            // TODO may just break? pseudo init state
            this->publish_state(guppy_msgs::srv::ChangeState::Request::TELEOP); // just move straight to teleop for sim purposes untill proper pipeline is created
        }

        void handle_holding() {
            if (this->holding_twist_.has_value())
                this->cmd_vel_publisher_->publish(holding_twist_.value());
        }

        void handle_nav() {
            if (this->nav_twist_.has_value())
                this->cmd_vel_publisher_->publish(nav_twist_.value());
        }

        void handle_task() {
            if (this->task_twist_.has_value())
                this->cmd_vel_publisher_->publish(task_twist_.value());
        }

        void handle_teleop() {
            if (this->teleop_twist_.has_value())
                this->cmd_vel_publisher_->publish(teleop_twist_.value());
        }

        void handle_disabled() {
            // TODO
        }

        void handle_fault() {
            // TODO
        }
        
        uint8_t current_state_;
        rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr state_publisher_;
        rclcpp::Service<guppy_msgs::srv::ChangeState>::SharedPtr state_service_;
        rclcpp::TimerBase::SharedPtr timer_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr holding_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr task_subscription_;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;

        std::optional<geometry_msgs::msg::Twist> holding_twist_;
        std::optional<geometry_msgs::msg::Twist> nav_twist_;
        std::optional<geometry_msgs::msg::Twist> task_twist_;
        std::optional<geometry_msgs::msg::Twist> teleop_twist_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<StateManager>();
    
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "State transition server ready.");

    rclcpp::spin(publisher_node);


    rclcpp::shutdown();

    return 0;
}
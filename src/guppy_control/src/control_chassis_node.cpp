#include "rclcpp/rclcpp.hpp"
#include "guppy_control/chassis_controller.hpp"
#include "guppy_control/t200_interface.hpp"

#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using namespace t200_interface;
using namespace chassis_controller;

class ControlChassis : public rclcpp::Node {
public:
  ControlChassis() : Node("control_chassis") {

    // declare parameters

    this->declare_parameter<std::vector<double>>("motor_positions", std::vector<double>(5 * N_MOTORS, 0.0)); // flattened 6xN
    this->declare_parameter<std::vector<double>>("motor_lower_bounds", std::vector<double>(N_MOTORS, 0.0));
    this->declare_parameter<std::vector<double>>("motor_upper_bounds", std::vector<double>(N_MOTORS, 0.0));
    this->declare_parameter<std::vector<double>>("axis_weight_matrix", std::vector<double>(6 * 6, 0.0)); // flattened 6 * 6
    this->declare_parameter<std::vector<double>>("pid_gains_vel_linear", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pid_gains_vel_angular", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pid_gains_pose_linear", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pid_gains_pose_angular", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("pose_lock_deadband", std::vector<double>(6, 0.0));
    this->declare_parameter<std::vector<double>>("drag_coefficients", std::vector<double>(6, 0.0));
    this->declare_parameter<std::vector<double>>("drag_areas", std::vector<double>(6, 0.0));
    this->declare_parameter<std::vector<double>>("drag_effect_matrix", std::vector<double>(6 * 6, 0.0)); // flattened 6x6
    this->declare_parameter<double>("water_density", 0.0);
    this->declare_parameter<double>("robot_volume", 0.0);
    this->declare_parameter<double>("robot_mass", 0.0);
    this->declare_parameter<std::vector<double>>("center_of_buoyancy", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<double>("qp_epsilon", 0.0);

    ChassisController::ChassisControllerParams parameters;

    load_params_(&parameters); // load parameters from configuration file

    this->param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this); // subscribe to parameter change event

    // parameter callback to update parameters on event

    auto param_callback = [this](const rcl_interfaces::msg::ParameterEvent & parameter_event) {
      RCLCPP_INFO(this->get_logger(), "Recieved parameter event from node for \"%s\"!", parameter_event.node.c_str());
      if (parameter_event.node != this->get_fully_qualified_name()) return;

      auto controller_params = this->controller->get_param_struct();

      auto param_map = get_param_map(&controller_params);

      bool update = false;

      for (const auto& param : parameter_event.changed_parameters) {
        const std::string name = rclcpp::Parameter::from_parameter_msg(param).get_name();
        const auto value = rclcpp::Parameter::from_parameter_msg(param);

        RCLCPP_INFO(this->get_logger(), "Parameter event recieved for '%s', ", parameter_event.node.c_str());

        auto pointer_it = param_map.find(name);
        auto transformer_it = transformers.find(name);
        auto printer_it = printers.find(name);

        std::visit([&](auto *pointer, auto& transformer) {
          using P = std::remove_cv_t<std::remove_reference_t<decltype(*pointer)>>;
          using R = std::remove_cv_t<std::remove_reference_t<std::invoke_result_t<decltype(transformer), const rclcpp::Parameter&>>>;
          
          if constexpr (std::is_same_v<P, R>) {
            std::string before, after;
            if (printer_it != printers.end())
              before = printer_it->second(static_cast<const void*>(pointer));
          
            auto tmp = transformer(value);
          
            if (printer_it != printers.end())
              after = printer_it->second(static_cast<const void*>(&tmp));
          
            *pointer = std::move(tmp);

            update = true;
          
            if (printer_it != printers.end())
              RCLCPP_INFO(this->get_logger(), "Parameter '%s' changed (%s)->(%s)", name.c_str(), before.c_str(), after.c_str());
            else
              RCLCPP_INFO(this->get_logger(), "Parameter '%s' updated.", name.c_str());
          }
        }, pointer_it->second, transformer_it->second);
      }

      if (update)
        this->controller->update_parameters(controller_params);
    };
    this->param_event_callback_handle_ = param_subscriber_->add_parameter_event_callback(param_callback);

    thruster_interface = new T200Interface("can0", {101, 102, 103, 104, 105, 106, 107, 108});

    controller = new ChassisController(parameters, thruster_interface, 100000);
    controller->start();

    // setup pub/sub/timer
    for (int i = 0; i < 8; i++) {
      sim_motor_publishers_[i] = this->create_publisher<std_msgs::msg::Float32>("/sim/motor_forces/m_" + std::to_string(i), 10);
    }

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&ControlChassis::odom_callback, this, std::placeholders::_1));
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,std::bind(&ControlChassis::cmdvel_callback, this, std::placeholders::_1));

    auto timer_callback = [this]() -> void {
      auto thrusts = controller->get_motor_thrusts();
      
      for (int i = 0; i < 8; i++) {
        std_msgs::msg::Float32 thrust;
        thrust.data = (double)thrusts[i];
        sim_motor_publishers_[i].get()->publish(thrust);
      }
    };

    timer_ = this->create_wall_timer(10ms, timer_callback);

    RCLCPP_INFO(this->get_logger(), "Setup parameters, thrust publishers, and subscribers.");
  }

  ~ControlChassis() {
    controller->stop();
    delete controller;
    delete thruster_interface;
  }

  void odom_callback(nav_msgs::msg::Odometry::SharedPtr msg) {
    controller->update_current_state(msg);
  }

  void cmdvel_callback(geometry_msgs::msg::Twist::SharedPtr msg) {
    controller->update_desired_state(msg);
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sim_motor_publishers_[8];
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;
  std::shared_ptr<rclcpp::ParameterEventCallbackHandle> param_event_callback_handle_;

  using ParameterPointerTypes = std::variant<
    Eigen::Matrix<double, 6, N_MOTORS>*,
    Eigen::Matrix<double, N_MOTORS, 1>*,
    std::vector<double>*,
    Eigen::Matrix<double, 6, 1>*,
    Eigen::Matrix<double, 6, 6>*,
    double*,
    Eigen::Matrix<double, 3, 1>*
  >;

  using ParameterTypes = std::variant<
    std::function<Eigen::Matrix<double, 6, N_MOTORS>(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, N_MOTORS, 1>(const rclcpp::Parameter&)>,
    std::function<std::vector<double>(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, 6, 6>(const rclcpp::Parameter&)>,
    std::function<double(const rclcpp::Parameter&)>,
    std::function<Eigen::Matrix<double, 3, 1>(const rclcpp::Parameter&)>
  >;

  std::unordered_map<std::string, ParameterTypes> transformers = {
    { 
      "motor_positions",
      std::function<Eigen::Matrix<double, 6, N_MOTORS>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {  
          return ControlChassis::to_motor_coefficients<N_MOTORS>(value.as_double_array());
        }
      )
    },
    {
      "motor_lower_bounds",
      std::function<Eigen::Matrix<double, N_MOTORS, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<N_MOTORS>(value.as_double_array());
        }
      )
    },
    { 
      "motor_upper_bounds",
      std::function<Eigen::Matrix<double, N_MOTORS, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<N_MOTORS>(value.as_double_array());
        }
      )
    },
    { 
      "axis_weight_matrix",
      std::function<Eigen::Matrix<double, 6, 6>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_matrix<6,6>(value.as_double_array());
        }
      )
    },
    { 
      "pid_gains_vel_linear",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double_array();
        }
      )
    },
    { 
      "pid_gains_vel_angular",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double_array();
        }
      )
    },
    { 
      "pid_gains_pose_linear",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) { return value.as_double_array();
        }
      )
    },
    { 
      "pid_gains_pose_angular",
      std::function<std::vector<double>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double_array();
        }
      )
    },
    { 
      "pose_lock_deadband",
      std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<6>(value.as_double_array());
        }
      )
    },
    { 
      "drag_coefficients",
      std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<6>(value.as_double_array());
        }
      )
    },
    { 
      "drag_areas",
      std::function<Eigen::Matrix<double, 6, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<6>(value.as_double_array());
        }
      )
    },
    { 
      "drag_effect_matrix",
      std::function<Eigen::Matrix<double, 6, 6>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_matrix<6,6>(value.as_double_array());
        }
      )
    },
    { 
      "water_density",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    },
    { 
      "robot_volume",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    },
    { 
      "robot_mass",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    },
    { 
      "center_of_buoyancy",
      std::function<Eigen::Matrix<double, 3, 1>(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return to_eigen_vec<3>(value.as_double_array());
        }
      )
    },
    { 
      "qp_epsilon",
      std::function<double(const rclcpp::Parameter&)>(
        [](const rclcpp::Parameter& value) {
          return value.as_double();
        }
      )
    }
  };
  
  inline static const Eigen::IOFormat LineFormat = Eigen::IOFormat(3, 0, ", ", "\n", "[", "]");
  inline static const Eigen::IOFormat InlineFormat = Eigen::IOFormat(3, 0, ", ", "", "", "");

  std::unordered_map<std::string, std::function<std::string(const void*)>> printers = {
    {
      "motor_positions",
      [](const void* pointer) {
        auto& matrix = *static_cast<const Eigen::Matrix<double,6,N_MOTORS>*>(pointer);
        return eigen_to_str(matrix, LineFormat);
      }
    },
    {
      "axis_weight_matrix",
      [](const void* pointer) {
        auto& matrix = *static_cast<const Eigen::Matrix<double,6,6>*>(pointer);
        return eigen_to_str(matrix, LineFormat);
      }
    },
    {
      "drag_effect_matrix",
      [](const void* pointer) {
        auto& matrix = *static_cast<const Eigen::Matrix<double,6,6>*>(pointer);
        return eigen_to_str(matrix, LineFormat);
      }
    },
    {
      "motor_lower_bounds",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,N_MOTORS,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "motor_upper_bounds",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,N_MOTORS,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "pose_lock_deadband",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,6,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "drag_coefficients",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,6,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "drag_areas",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,6,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "center_of_buoyancy",
      [](const void* pointer) {
        auto& vec = *static_cast<const Eigen::Matrix<double,3,1>*>(pointer);
        return eigen_to_str(vec, InlineFormat);
      }
    },
    {
      "pid_gains_vel_linear",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "pid_gains_vel_angular",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "pid_gains_pose_linear",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "pid_gains_pose_angular",
      [](const void* pointer) {
        return vec_to_str(*static_cast<const std::vector<double>*>(pointer));
      }
    },
    {
      "water_density",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    },
    {
      "robot_volume",
      [](const void* pointer) { 
        return std::to_string(*static_cast<const double*>(pointer));
      }
    },
    {
      "robot_mass",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    },
    {
      "qp_epsilon",
      [](const void* pointer) {
        return std::to_string(*static_cast<const double*>(pointer));
      }
    }
  };
  
  T200Interface* thruster_interface;
  ChassisController* controller;

  void load_params_(ChassisController::ChassisControllerParams* params) {
    auto param_map = get_param_map(params);

    for (auto& [name, pointer_to] : param_map) {
      if (!this->has_parameter(name)) {
        RCLCPP_WARN(this->get_logger(), "Parameter '%s' not set, skipping.", name.c_str());
        continue;
      }

      const auto param = this->get_parameter(name);

      auto transformer_it = transformers.find(name);

      std::visit([&](auto* pointer, auto& transformer) {
        using Pointee = std::remove_cv_t<std::remove_reference_t<decltype(*pointer)>>;
        using Ret = std::invoke_result_t<decltype(transformer), const rclcpp::Parameter&>;
        if constexpr (std::is_same_v<Pointee, Ret>)
          *pointer = transformer(param);
      }, pointer_to, transformer_it->second);
    }
  }

  void print_params_struct_(ChassisController::ChassisControllerParams* params) {
    auto param_map = get_param_map(params);
    
    for (auto& [name, pointer_to] : param_map) {
      const auto it = printers.find(name);
      if (it == printers.end()) {
        RCLCPP_WARN(this->get_logger(), "No printer registered for '%s'", name.c_str());
        continue;
      }
      
      std::string rendered;
      std::visit(
        [&](auto* pointer) {
          rendered = it->second(static_cast<const void*>(pointer));
        }, pointer_to
      );

      RCLCPP_INFO(this->get_logger(), "%s: %s", name.c_str(), rendered.c_str());
    }
  }

  void print_params_(void) {
    ChassisController::ChassisControllerParams params;

    load_params_(&params);

    print_params_struct_(&params);
  }

  static std::string vec_to_str(const std::vector<double>& vec) {
    std::ostringstream oss;

    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      if (i) oss << ", ";
      oss << vec[i];
    }

    oss << "]";

    return oss.str();
  }

  template <typename Derived>
  static std::string eigen_to_str(const Eigen::MatrixBase<Derived>& matrix, const Eigen::IOFormat& format) {
    std::ostringstream oss;

    oss << matrix.format(format);

    return oss.str();
  }

  template <int N>
  static Eigen::Matrix<double, N, 1> to_eigen_vec(const std::vector<double>& vec) {
    if (vec.size() != N) {
      //RCLCPP_ERROR(this->get_logger(), "Bad vector parameter passed to controller!");
      throw std::runtime_error("Bad parameter passed to controller.");
    }

    return Eigen::Map<const Eigen::Matrix<double, N, 1>>(vec.data());
  }

  template <int R, int C>
  static Eigen::Matrix<double, R, C> to_eigen_matrix(const std::vector<double>& vec) {
    if (vec.size() != R * C) {
      //RCLCPP_ERROR(this->get_logger(), "Bad matrix parameter passed to controller!");
      throw std::runtime_error("Bad matrxc parameter passed to controller.");
    }

    return Eigen::Map<const Eigen::Matrix<double, R, C, Eigen::RowMajor>>(vec.data());
  }

  template <int N>
  static Eigen::Matrix<double, 6, N> to_motor_coefficients(const std::vector<double>& flat5N) {
    Eigen::Matrix<double, 6, N> M;
    for (int i = 0; i < N; ++i) {
      const double x     = flat5N[5*i + 0];
      const double y     = flat5N[5*i + 1];
      const double z     = flat5N[5*i + 2];
      const double phi   = flat5N[5*i + 3];
      const double theta = flat5N[5*i + 4];
      M.col(i) = get_single_motor_coefficients(x, y, z, phi, theta);
    }

    return M;
  }

  std::unordered_map<std::string, ParameterPointerTypes> get_param_map(ChassisController::ChassisControllerParams* params) {
    return {
      { "motor_positions",        &params->motor_coefficients },
      { "motor_lower_bounds",     &params->motor_lower_bounds },
      { "motor_upper_bounds",     &params->motor_upper_bounds },
      { "axis_weight_matrix",     &params->axis_weight_matrix },
      { "pid_gains_vel_linear",   &params->pid_gains_vel_linear },
      { "pid_gains_vel_angular",  &params->pid_gains_vel_angular },
      { "pid_gains_pose_linear",  &params->pid_gains_pose_linear },
      { "pid_gains_pose_angular", &params->pid_gains_pose_angular },
      { "pose_lock_deadband",     &params->pose_lock_deadband },
      { "drag_coefficients",      &params->drag_coefficients },
      { "drag_areas",             &params->drag_areas },
      { "drag_effect_matrix",     &params->drag_effect_matrix },
      { "water_density",          &params->water_density },
      { "robot_volume",           &params->robot_volume },
      { "robot_mass",             &params->robot_mass },
      { "center_of_buoyancy",     &params->center_of_buoyancy },
      { "qp_epsilon",             &params->qp_epsilon }
    };
  }

  /*
    @brief get the motor coefficients from a thruster's pose
    @param x the x loctaion in meters of the thruster
    @param y the y loctaion in meters of the thruster
    @param z the z loctaion in meters of the thruster
    @param phi the phi of the thruster in spherical coordinates (degrees, positive up)
    @param theta the theta of the thruster in spherical coordinates (degrees)
  */
  static Eigen::Matrix<double, 6, 1> get_single_motor_coefficients(double x, double y, double z, double phi, double theta) {
    Eigen::Matrix<double, 6, 1> out;

    // convert to rads
    double p = (90 - phi) * (M_PI / 180);
    double t = (270 + theta) * (M_PI / 180);
    // calculate to reuse
    double sinp = sin(p);
    double sint = sin(t);
    double cost = cos(t);
    double cosp = cos(p);

    out <<\
        sinp * cost,                  \
        sinp * sint,                  \
        cosp,                         \
        (z*sinp*sint) - (y*cosp),     \
        (x*cosp) - (z*sinp*cost),     \
        (y*sinp*cost) - (x*sinp*sint);
    return out;
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<ControlChassis>());

  rclcpp::shutdown();

  return 0;
}
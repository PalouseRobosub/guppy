#include <memory>
#include "rclcpp/rclcpp.hpp"

class TorpedoService : public rclcpp::Node {
public:
    TorpedoService() : Node("torpedo_srv") {
        
    };
    
private:
    
};

int main(const int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    const auto node = std::make_shared<TorpedoService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
// drone_dynamics placeholder node.
// Step 2 将替换为完整动力学积分：电机一阶、推力/力矩 EOM、odom/imu/path/tf 发布。
#include <rclcpp/rclcpp.hpp>

class DynamicsPlaceholder : public rclcpp::Node {
public:
  DynamicsPlaceholder() : rclcpp::Node("drone_dynamics") {
    RCLCPP_INFO(this->get_logger(), "drone_dynamics placeholder ready (Step 0 skeleton)");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicsPlaceholder>());
  rclcpp::shutdown();
  return 0;
}
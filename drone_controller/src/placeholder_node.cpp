// drone_controller placeholder node.
// Step 3 将替换为级联 PD 位置→姿态→mixer 逆→RPM。
#include <rclcpp/rclcpp.hpp>
class ControllerPlaceholder : public rclcpp::Node {
public:
  ControllerPlaceholder() : rclcpp::Node("drone_controller") {
    RCLCPP_INFO(this->get_logger(), "drone_controller placeholder ready (Step 0 skeleton)");
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerPlaceholder>());
  rclcpp::shutdown();
  return 0;
}

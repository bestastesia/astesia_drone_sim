// drone_planner placeholder node.
// Step 6 将替换为 2D 膨胀栅格 A* + safe_goal 发布。
#include <rclcpp/rclcpp.hpp>
class PlannerPlaceholder : public rclcpp::Node {
public:
  PlannerPlaceholder() : rclcpp::Node("drone_planner") {
    RCLCPP_INFO(this->get_logger(), "drone_planner placeholder ready (Step 0 skeleton)");
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerPlaceholder>());
  rclcpp::shutdown();
  return 0;
}

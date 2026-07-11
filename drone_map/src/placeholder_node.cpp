// drone_map placeholder node.
// Step 5 将替换为确定性障碍物生成 + MarkerArray 发布。
#include <rclcpp/rclcpp.hpp>
class MapPlaceholder : public rclcpp::Node {
public:
  MapPlaceholder() : rclcpp::Node("drone_map") {
    RCLCPP_INFO(this->get_logger(), "drone_map placeholder ready (Step 0 skeleton)");
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPlaceholder>());
  rclcpp::shutdown();
  return 0;
}

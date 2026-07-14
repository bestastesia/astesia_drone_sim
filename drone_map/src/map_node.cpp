// map_node.cpp — 静态障碍物生成 + MarkerArray 发布
// 支持显式 YAML 列表（demo 用，完全可复现）或程序随机生成（固定 seed）
// 发布 /map/obstacles (visualization_msgs/MarkerArray), 1 Hz, frame=map

#include <memory>
#include <vector>
#include <random>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "drone_common/obstacle.hpp"

using drone_common::Obstacle;
using drone_common::Shape;

class MapNode : public rclcpp::Node {
 public:
  MapNode() : Node("drone_map") {
    declare_parameter<bool>("procedural", false);
    declare_parameter<int>("seed", 42);
    declare_parameter<std::vector<double>>("bounds", {-2.0, 4.0, -2.0, 4.0, 0.0, 5.0});
    declare_parameter<int>("num_obstacles", 8);
    declare_parameter<double>("clear_radius", 0.8);
    declare_parameter<double>("size_min", 0.2);
    declare_parameter<double>("size_max", 0.5);
    declare_parameter<std::vector<std::string>>("obstacles",
        {"sphere 0.0 0.0 1.5 0.3"});  // 默认占位，YAML 会覆盖
    declare_parameter<std::string>("frame_id", "map");

    bool procedural = get_parameter("procedural").as_bool();
    if (procedural) {
      obstacles_ = generateProcedural();
    } else {
      obstacles_ = loadExplicit();
    }
    if (obstacles_.empty()) {
      // fallback
      Obstacle o; o.shape = Shape::Sphere; o.center << 0, 0, 1.5; o.radius = 0.3;
      obstacles_.push_back(o);
    }

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/map/obstacles", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish(); });
    first_publish_ = true;

    RCLCPP_INFO(get_logger(), "drone_map ready: %zu obstacles", obstacles_.size());
  }

 private:
  std::vector<Obstacle> loadExplicit() {
    std::vector<Obstacle> obs;
    auto raw = get_parameter("obstacles").as_string_array();
    if (raw.empty()) {
      RCLCPP_WARN(get_logger(), "obstacles list empty, falling back to procedural");
      return generateProcedural();
    }
    for (const auto& s : raw) {
      // 格式："sphere 0.8 0.3 1.5 0.3" 或 "cylinder 1.2 0.8 0.9 0.25 1.0" 或 "cube 1.0 -0.2 1.5 0.2 0.2 0.3"
      Obstacle o;
      std::istringstream iss(s);
      std::string shape; iss >> shape;
      if (shape == "sphere") {
        o.shape = Shape::Sphere;
        iss >> o.center.x() >> o.center.y() >> o.center.z() >> o.radius;
      } else if (shape == "cylinder") {
        o.shape = Shape::Cylinder;
        iss >> o.center.x() >> o.center.y() >> o.center.z() >> o.radius >> o.height;
      } else if (shape == "cube") {
        o.shape = Shape::Cube;
        iss >> o.center.x() >> o.center.y() >> o.center.z()
            >> o.half_extents.x() >> o.half_extents.y() >> o.half_extents.z();
      } else {
        continue;
      }
      obs.push_back(o);
    }
    return obs;
  }

  std::vector<Obstacle> generateProcedural() {
    int seed = get_parameter("seed").as_int();
    auto bounds = get_parameter("bounds").as_double_array();
    int num = get_parameter("num_obstacles").as_int();
    double clear_r = get_parameter("clear_radius").as_double();
    double smin = get_parameter("size_min").as_double();
    double smax = get_parameter("size_max").as_double();
    if (bounds.size() < 6) return {};

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dx(bounds[0], bounds[1]);
    std::uniform_real_distribution<double> dy(bounds[2], bounds[3]);
    std::uniform_real_distribution<double> dz(0.5, 2.5);
    std::uniform_real_distribution<double> ds(smin, smax);
    std::uniform_int_distribution<int> shape_rng(0, 2);

    std::vector<Obstacle> obs;
    int attempts = 0;
    while ((int)obs.size() < num && attempts < 500) {
      ++attempts;
      Obstacle o;
      o.center << dx(rng), dy(rng), dz(rng);
      double s = ds(rng);
      int st = shape_rng(rng);
      if (st == 0) { o.shape = Shape::Sphere; o.radius = s; }
      else if (st == 1) { o.shape = Shape::Cylinder; o.radius = s; o.height = 2.0; }
      else { o.shape = Shape::Cube; o.half_extents << s, s, s * 0.5; }
      // reject near start (0,0,1.5) or goal (2,1,1.5)
      if ((o.center - Eigen::Vector3d(0, 0, 1.5)).norm() < clear_r) continue;
      if ((o.center - Eigen::Vector3d(2, 1, 1.5)).norm() < clear_r) continue;
      // reject overlap with existing
      bool overlap = false;
      for (auto& p : obs) {
        if ((o.center - p.center).norm() < (s + (p.shape == Shape::Cube ? p.half_extents.norm() : p.radius))) {
          overlap = true; break;
        }
      }
      if (!overlap) obs.push_back(o);
    }
    return obs;
  }

  void publish() {
    visualization_msgs::msg::MarkerArray arr;
    if (first_publish_) {
      // DELETEALL first to clean any stale markers
      visualization_msgs::msg::Marker del;
      del.action = visualization_msgs::msg::Marker::DELETEALL;
      arr.markers.push_back(del);
      first_publish_ = false;
    }
    int id = 0;
    for (const auto& o : obstacles_) {
      visualization_msgs::msg::Marker m;
      m.header.stamp = now();
      m.header.frame_id = get_parameter("frame_id").as_string();
      m.ns = "obstacles";
      m.id = id++;
      m.action = visualization_msgs::msg::Marker::ADD;
      m.lifetime = rclcpp::Duration(0, 0);
      m.color.r = 1.0f; m.color.g = 0.5f; m.color.b = 0.0f; m.color.a = 0.6f;
      m.pose.position.x = o.center.x();
      m.pose.position.y = o.center.y();
      m.pose.position.z = o.center.z();
      m.pose.orientation.w = 1.0;
      switch (o.shape) {
        case Shape::Sphere:
          m.type = visualization_msgs::msg::Marker::SPHERE;
          m.scale.x = m.scale.y = m.scale.z = o.radius * 2;
          break;
        case Shape::Cylinder:
          m.type = visualization_msgs::msg::Marker::CYLINDER;
          m.scale.x = m.scale.y = o.radius * 2;
          m.scale.z = o.height;
          break;
        case Shape::Cube:
          m.type = visualization_msgs::msg::Marker::CUBE;
          m.scale.x = o.half_extents.x() * 2;
          m.scale.y = o.half_extents.y() * 2;
          m.scale.z = o.half_extents.z() * 2;
          break;
      }
      arr.markers.push_back(m);
    }
    pub_->publish(arr);
  }

  std::vector<Obstacle> obstacles_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool first_publish_ = true;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapNode>());
  rclcpp::shutdown();
  return 0;
}
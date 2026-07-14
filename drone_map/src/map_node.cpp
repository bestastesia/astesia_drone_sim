// map_node.cpp — 静态障碍物生成 + MarkerArray 发布
// 支持显式 YAML 列表（demo 用，完全可复现）或程序随机生成（固定 seed）
// 发布 /map/obstacles (visualization_msgs/MarkerArray), 1 Hz, frame=map

#include <memory>
#include <vector>
#include <random>
#include <string>
#include <sstream>

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
    declare_parameter<int>("num_spheres", -1);     // -1 = 不限制（随机分配）
    declare_parameter<int>("num_cylinders", -1);
    declare_parameter<int>("num_cubes", -1);
    declare_parameter<double>("clear_radius", 0.8);
    declare_parameter<double>("size_min", 0.2);
    declare_parameter<double>("size_max", 0.5);
    declare_parameter<std::vector<std::string>>("obstacles",
        {"sphere 0.0 0.0 1.5 0.3"});  // 默认占位，YAML 会覆盖
    declare_parameter<std::string>("frame_id", "map");

    obstacles_ = generateFromParams();

    pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/map/obstacles", 10);
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() { publish(); });
    first_publish_ = true;

    // 运行时参数变更回调——支持 ros2 param set 立即重新生成障碍物
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          return onParamChange(params);
        });

    RCLCPP_INFO(get_logger(), "drone_map ready: %zu obstacles, procedural=%s seed=%d",
        obstacles_.size(),
        get_parameter("procedural").as_bool() ? "true" : "false",
        get_parameter("seed").as_int());
  }

 private:
  std::vector<Obstacle> generateFromParams() {
    bool procedural = get_parameter("procedural").as_bool();
    std::vector<Obstacle> obs;
    if (procedural) {
      obs = generateProcedural();
    } else {
      obs = loadExplicit();
    }
    if (obs.empty()) {
      Obstacle o; o.shape = Shape::Sphere; o.center << 0, 0, 1.5; o.radius = 0.3;
      obs.push_back(o);
    }
    return obs;
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
      const std::vector<rclcpp::Parameter>& params) {
    bool need_regen = false;
    for (const auto& p : params) {
      const auto& n = p.get_name();
      if (n == "procedural" || n == "seed" || n == "num_obstacles" ||
          n == "bounds" || n == "clear_radius" || n == "size_min" ||
          n == "size_max" || n == "obstacles" ||
          n == "num_spheres" || n == "num_cylinders" || n == "num_cubes") {
        need_regen = true;
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    if (need_regen) {
      obstacles_ = generateFromParams();
      first_publish_ = true;  // 下一帧发 DELETEALL 清旧 Marker
      RCLCPP_INFO(get_logger(), "regenerated: %zu obstacles", obstacles_.size());
    }
    return result;
  }
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
    int n_spheres   = get_parameter("num_spheres").as_int();
    int n_cylinders = get_parameter("num_cylinders").as_int();
    int n_cubes     = get_parameter("num_cubes").as_int();
    double clear_r = get_parameter("clear_radius").as_double();
    double smin = get_parameter("size_min").as_double();
    double smax = get_parameter("size_max").as_double();
    if (bounds.size() < 6) return {};

    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> dx(bounds[0], bounds[1]);
    std::uniform_real_distribution<double> dy(bounds[2], bounds[3]);
    std::uniform_real_distribution<double> dz(0.5, 2.5);
    std::uniform_real_distribution<double> ds(smin, smax);

    // 建立形状队列：先放指定数量的每种，剩余随机分配
    std::vector<Shape> shape_queue;
    int ns = (n_spheres   >= 0) ? n_spheres   : num;   // -1 = 不限
    int nc = (n_cylinders >= 0) ? n_cylinders : num;
    int nb = (n_cubes     >= 0) ? n_cubes     : num;
    for (int i = 0; i < ns; ++i) shape_queue.push_back(Shape::Sphere);
    for (int i = 0; i < nc; ++i) shape_queue.push_back(Shape::Cylinder);
    for (int i = 0; i < nb; ++i) shape_queue.push_back(Shape::Cube);

    // 补充到至少 num 个（用随机填充）
    std::uniform_int_distribution<int> shape_rng(0, 2);
    while ((int)shape_queue.size() < num) {
      int st = shape_rng(rng);
      if (st == 0) shape_queue.push_back(Shape::Sphere);
      else if (st == 1) shape_queue.push_back(Shape::Cylinder);
      else shape_queue.push_back(Shape::Cube);
    }

    std::vector<Obstacle> obs;
    int attempts = 0;
    size_t sq_idx = 0;
    while ((int)obs.size() < num && attempts < 1000) {
      ++attempts;
      Obstacle o;
      o.center << dx(rng), dy(rng), dz(rng);
      double s = ds(rng);
      o.shape = shape_queue[sq_idx % shape_queue.size()];
      ++sq_idx;
      if (o.shape == Shape::Sphere) { o.radius = s; }
      else if (o.shape == Shape::Cylinder) { o.radius = s; o.height = 2.0; }
      else { o.half_extents << s, s, s * 0.5; }
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
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapNode>());
  rclcpp::shutdown();
  return 0;
}
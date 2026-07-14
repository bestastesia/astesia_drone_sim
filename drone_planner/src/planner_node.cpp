// planner_node.cpp — 2D 膨胀栅格 A* 路径规划 + 安全局部目标发布
//
// 输入 /drone/odom + /drone/goal + /map/obstacles
// 输出 /drone/safe_goal (前方 lookahead 处的 waypoint)
//       /drone/planned_path (完整 A* 路径)
//       /drone/planner_status (std_msgs/String: OK/NO_PATH/GOAL_IN_OBSTACLE/START_IN_OBSTACLE)

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "drone_common/obstacle.hpp"
#include "drone_planner/grid.hpp"
#include "drone_planner/astar.hpp"

using drone_common::Obstacle;
using drone_common::Shape;

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("drone_planner") {
    declare_parameter<double>("resolution", 0.1);
    declare_parameter<std::vector<double>>("bounds", {-2.0, 4.0, -2.0, 4.0});
    declare_parameter<double>("safety_distance", 0.4);
    declare_parameter<double>("drone_radius", 0.2);
    declare_parameter<bool>("use_goal_z", true);
    declare_parameter<double>("cruise_altitude", 1.5);
    declare_parameter<double>("default_hover_z", 1.5);  // goal z<0.5 时用这个替代
    declare_parameter<double>("replan_rate", 1.0);
    declare_parameter<double>("lookahead", 0.5);
    declare_parameter<double>("advance_tol", 0.3);
    declare_parameter<std::string>("grid_frame", "map");

    resolution_  = get_parameter("resolution").as_double();
    auto b = get_parameter("bounds").as_double_array();
    bounds_ = {b[0], b[1], b[2], b[3]};
    safety_dist_ = get_parameter("safety_distance").as_double();
    drone_r_     = get_parameter("drone_radius").as_double();
    use_goal_z_  = get_parameter("use_goal_z").as_bool();
    cruise_z_    = get_parameter("cruise_altitude").as_double();
    lookahead_   = get_parameter("lookahead").as_double();
    advance_tol_ = get_parameter("advance_tol").as_double();
    grid_frame_  = get_parameter("grid_frame").as_string();

    double replan_rate = get_parameter("replan_rate").as_double();
    int period_ms = static_cast<int>(1000.0 / replan_rate);

    // 订阅
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_); last_odom_ = m; });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone/goal", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_); last_goal_ = m; });
    obs_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        "/map/obstacles", 10,
        [this](const visualization_msgs::msg::MarkerArray::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_); last_obs_ = m; });

    safe_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/drone/safe_goal", 10);
    path_pub_      = create_publisher<nav_msgs::msg::Path>("/drone/planned_path", 10);
    status_pub_    = create_publisher<std_msgs::msg::String>("/drone/planner_status", 10);

    replan_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms), [this]() { replan(); });

    RCLCPP_INFO(get_logger(), "planner ready: res=%.2f safety=%.2f lookahead=%.2f",
                resolution_, safety_dist_, lookahead_);
  }

 private:
  void replan() {
    nav_msgs::msg::Odometry::SharedPtr odom;
    geometry_msgs::msg::PoseStamped::SharedPtr goal;
    visualization_msgs::msg::MarkerArray::SharedPtr obs;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      odom = last_odom_; goal = last_goal_; obs = last_obs_;
    }
    if (!odom) { publishStatus("NO_ODOM"); return; }

    double px = odom->pose.pose.position.x;
    double py = odom->pose.pose.position.y;
    double pz = odom->pose.pose.position.z;

    // 目标默认
    double gx = px, gy = py, gz = cruise_z_;
    if (goal) {
      gx = goal->pose.position.x;
      gy = goal->pose.position.y;
      gz = goal->pose.position.z;
      if (gz < 0.5) gz = get_parameter("default_hover_z").as_double();  // 2D Goal Pose fix
    }
    double z_cruise = use_goal_z_ ? gz : cruise_z_;

    // 构建障碍物列表
    std::vector<Obstacle> obstacles;
    if (obs) {
      for (const auto& m : obs->markers) {
        if (m.action == m.DELETE || m.ns != "obstacles") continue;
        Obstacle o;
        o.center << m.pose.position.x, m.pose.position.y, m.pose.position.z;
        switch (m.type) {
          case visualization_msgs::msg::Marker::SPHERE:
            o.shape = Shape::Sphere; o.radius = m.scale.x * 0.5; break;
          case visualization_msgs::msg::Marker::CYLINDER:
            o.shape = Shape::Cylinder; o.radius = m.scale.x * 0.5; o.height = m.scale.z; break;
          case visualization_msgs::msg::Marker::CUBE:
            o.shape = Shape::Cube;
            o.half_extents << m.scale.x * 0.5, m.scale.y * 0.5, m.scale.z * 0.5; break;
          default: continue;
        }
        obstacles.push_back(o);
      }
    }
    if (obstacles.empty()) {
      // 还没收到障碍物 → 不要直冲目标，悬停当前位姿等待
      if (!obs) {
        publishStatus("WAITING_FOR_MAP");
        publishSafeGoal(px, py, cruise_z_);  // hold position
        return;
      }
      // 收到过 MarkerArray 但解析为空 → 真无障碍，可直飞
      publishSafeGoal(gx, gy, z_cruise);
      publishStatus("OK_NO_OBSTACLES");
      return;
    }

    // 构建膨胀栅格
    double inflate = safety_dist_ + drone_r_;
    auto grid = drone_planner::buildOccupancyGrid(
        obstacles, resolution_, bounds_[0], bounds_[1],
        bounds_[2], bounds_[3], inflate, z_cruise);
    int occupied = 0;
    for (auto v : grid.cells) if (v) ++occupied;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "replan: %zu obstacles, grid %dx%d, %d occupied cells, z_cruise=%.2f",
        obstacles.size(), grid.nx, grid.ny, occupied, z_cruise);

    // A*（astarSearch 内部已有 start/goal 的 BFS 就近 free cell 处理）
    auto path = drone_planner::astarSearch(grid, px, py, gx, gy);
    if (path.empty()) {
      publishStatus("NO_PATH");
      return;
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
        "planned_path: %zu waypoints (smoothed)", path.size());

    // 发布计划路径
    publishPath(path, z_cruise);

    // 找到当前位置在整条路径上最近的航点（从头搜，不依赖跨 replan 的旧 index）
    double best_dist = 1e9; int closest_i = 0;
    for (int i = 0; i < static_cast<int>(path.size()); ++i) {
      double d = std::hypot(path[i].x() - px, path[i].y() - py);
      if (d < best_dist) { best_dist = d; closest_i = i; }
    }

    // 单调推进：safe_goal 永远取 closest+1，但引入 last_look_i 保证不回退
    int candidate = std::min(closest_i + 1, static_cast<int>(path.size()) - 1);
    if (candidate > last_look_i_) last_look_i_ = candidate;
    else if (closest_i >= static_cast<int>(path.size()) - 1)
      last_look_i_ = static_cast<int>(path.size()) - 1;  // at goal, stay

    publishSafeGoal(path[last_look_i_].x(), path[last_look_i_].y(), z_cruise);

    // 到达最终目标后不再更新
    if (last_look_i_ < static_cast<int>(path.size()) - 1) {
      publishStatus("OK");
    } else {
      publishStatus("AT_GOAL");
    }
  }

  void publishStatus(const std::string& s) {
    auto msg = std_msgs::msg::String(); msg.data = s;
    status_pub_->publish(msg);
  }

  void publishSafeGoal(double x, double y, double z) {
    auto msg = geometry_msgs::msg::PoseStamped();
    msg.header.stamp = now(); msg.header.frame_id = grid_frame_;
    msg.pose.position.x = x; msg.pose.position.y = y; msg.pose.position.z = z;
    msg.pose.orientation.w = 1.0;
    safe_goal_pub_->publish(msg);
  }

  void publishPath(const std::vector<Eigen::Vector2d>& path, double z) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now(); msg.header.frame_id = grid_frame_;
    for (const auto& wp : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = grid_frame_;
      ps.pose.position.x = wp.x(); ps.pose.position.y = wp.y(); ps.pose.position.z = z;
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
  }

  // 参数
  double resolution_, safety_dist_, drone_r_, cruise_z_, lookahead_, advance_tol_;
  bool use_goal_z_;
  std::vector<double> bounds_;
  std::string grid_frame_;

  int last_look_i_ = 0;

  std::mutex mtx_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_goal_;
  visualization_msgs::msg::MarkerArray::SharedPtr last_obs_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr obs_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr safe_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
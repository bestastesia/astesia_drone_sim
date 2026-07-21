// planner_node.cpp — 四模式感知/规划 (2D×3D × GLOBAL×FOV)
//
// 输入 /drone/odom + /drone/goal + /map/obstacles
// 输出 /drone/safe_goal (前方 waypoint)
//       /drone/planned_path (完整 A* 路径)
//       /drone/planner_status
//       /drone/fov_cone (TRIANGLE_LIST — FOV 锥体)
//       /drone/fov_voxels (CUBE_LIST — FOV 可见体素)

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/msg/string.hpp"
#include "drone_common/obstacle.hpp"
#include "drone_common/voxel.hpp"
#include "drone_planner/grid.hpp"
#include "drone_planner/astar.hpp"
#include "drone_planner/astar3d.hpp"
#include "drone_planner/perception.hpp"

using drone_common::Obstacle;
using drone_common::Shape;
using drone_common::Vec3d;
using drone_common::Vec3i;

class PlannerNode : public rclcpp::Node {
 public:
  PlannerNode() : Node("drone_planner") {
    // ---- 基础参数 ----
    declare_parameter<double>("resolution", 0.1);
    declare_parameter<std::vector<double>>("bounds", {-2.0, 4.0, -2.0, 4.0});
    declare_parameter<double>("safety_distance", 0.4);
    declare_parameter<double>("drone_radius", 0.2);
    declare_parameter<bool>("use_goal_z", true);
    declare_parameter<double>("cruise_altitude", 1.5);
    declare_parameter<double>("default_hover_z", 1.5);
    declare_parameter<double>("replan_rate", 1.0);
    declare_parameter<double>("lookahead", 0.5);
    declare_parameter<double>("advance_tol", 0.3);
    declare_parameter<std::string>("grid_frame", "map");

    // ---- 感知参数 (热切换) ----
    declare_parameter<std::string>("perception_mode", "global");  // "global" | "fov"
    declare_parameter<std::string>("planner_dim", "2d");          // "2d" | "3d"
    declare_parameter<double>("fov_h_deg", 60.0);   // 水平视场角 (°)
    declare_parameter<double>("fov_v_deg", 45.0);   // 垂直视场角 (°)
    declare_parameter<double>("fov_range", 5.0);    // 感知距离 m
    declare_parameter<double>("voxel_resolution", 0.2); // 体素分辨率 m
    declare_parameter<std::vector<double>>("z_bounds", {0.0, 4.0}); // 体素 z 范围

    loadBaseParams();
    loadPerceptionConfig();

    double replan_rate = get_parameter("replan_rate").as_double();
    int period_ms = static_cast<int>(1000.0 / replan_rate);

    // 感知引擎
    engine_ = std::make_unique<drone_planner::PerceptionEngine>(perc_cfg_);

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

    safe_goal_pub_  = create_publisher<geometry_msgs::msg::PoseStamped>("/drone/safe_goal", 10);
    path_pub_       = create_publisher<nav_msgs::msg::Path>("/drone/planned_path", 10);
    status_pub_     = create_publisher<std_msgs::msg::String>("/drone/planner_status", 10);
    fov_cone_pub_   = create_publisher<visualization_msgs::msg::Marker>("/drone/fov_cone", 10);
    fov_voxels_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/drone/fov_voxels", 10);

    replan_timer_ = create_wall_timer(
        std::chrono::milliseconds(period_ms), [this]() { replan(); });

    // 参数变更回调（热切换感知模式）
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          for (const auto& p : params) {
            if (p.get_name() == "perception_mode") {
              perc_cfg_.mode = (p.as_string() == "fov")
                  ? drone_planner::PerceptionMode::FOV
                  : drone_planner::PerceptionMode::GLOBAL;
              RCLCPP_INFO(get_logger(), "perception_mode := %s", p.as_string().c_str());
            } else if (p.get_name() == "planner_dim") {
              perc_cfg_.dim = (p.as_string() == "3d")
                  ? drone_planner::PlannerDim::THREE_D
                  : drone_planner::PlannerDim::TWO_D;
              RCLCPP_INFO(get_logger(), "planner_dim := %s", p.as_string().c_str());
            } else if (p.get_name() == "fov_h_deg") {
              perc_cfg_.fov_h_rad = p.as_double() * M_PI / 180.0;
            } else if (p.get_name() == "fov_v_deg") {
              perc_cfg_.fov_v_rad = p.as_double() * M_PI / 180.0;
            } else if (p.get_name() == "fov_range") {
              perc_cfg_.fov_range = p.as_double();
            } else if (p.get_name() == "voxel_resolution") {
              perc_cfg_.voxel_res = p.as_double();
            } else if (p.get_name() == "z_bounds") {
              auto z = p.as_double_array();
              if (z.size() >= 2) { perc_cfg_.z_min = z[0]; perc_cfg_.z_max = z[1]; }
            }
          }
          rcl_interfaces::msg::SetParametersResult r; r.successful = true; return r;
        });

    RCLCPP_INFO(get_logger(),
        "planner ready: mode=%s dim=%s res=%.2f safety=%.2f voxel_res=%.2f",
        (perc_cfg_.mode == drone_planner::PerceptionMode::FOV ? "fov" : "global"),
        (perc_cfg_.dim == drone_planner::PlannerDim::THREE_D ? "3d" : "2d"),
        resolution_, safety_dist_, perc_cfg_.voxel_res);
  }

 private:
  void loadBaseParams() {
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
  }

  void loadPerceptionConfig() {
    perc_cfg_.mode = (get_parameter("perception_mode").as_string() == "fov")
        ? drone_planner::PerceptionMode::FOV : drone_planner::PerceptionMode::GLOBAL;
    perc_cfg_.dim = (get_parameter("planner_dim").as_string() == "3d")
        ? drone_planner::PlannerDim::THREE_D : drone_planner::PlannerDim::TWO_D;
    perc_cfg_.fov_h_rad = get_parameter("fov_h_deg").as_double() * M_PI / 180.0;
    perc_cfg_.fov_v_rad = get_parameter("fov_v_deg").as_double() * M_PI / 180.0;
    perc_cfg_.fov_range = get_parameter("fov_range").as_double();
    perc_cfg_.voxel_res = get_parameter("voxel_resolution").as_double();
    auto z = get_parameter("z_bounds").as_double_array();
    if (z.size() >= 2) { perc_cfg_.z_min = z[0]; perc_cfg_.z_max = z[1]; }
  }

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

    // 无人机 forward（从四元数提取机身 +x），默认 (1,0,0)
    Vec3d drone_pos(px, py, pz);
    Vec3d drone_fwd(1, 0, 0);
    {
      double qw = odom->pose.pose.orientation.w;
      double qx = odom->pose.pose.orientation.x;
      double qy = odom->pose.pose.orientation.y;
      double qz = odom->pose.pose.orientation.z;
      Eigen::Quaterniond q(qw, qx, qy, qz);
      if (std::isfinite(qw) && q.norm() > 1e-9) {
        q.normalize();
        drone_fwd = q.toRotationMatrix().col(0);  // 机身 +x
      }
    }

    // 目标默认
    double gx = px, gy = py, gz = cruise_z_;
    if (goal) {
      gx = goal->pose.position.x;
      gy = goal->pose.position.y;
      gz = goal->pose.position.z;
      if (gz < 0.5) gz = get_parameter("default_hover_z").as_double();
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
      if (!obs) { publishStatus("WAITING_FOR_MAP");
        publishSafeGoal(px, py, cruise_z_); return; }
      publishSafeGoal(gx, gy, z_cruise);
      publishStatus("OK_NO_OBSTACLES");
      return;
    }

    double inflate = safety_dist_ + drone_r_;

    // ===== 四模式路由 =====
    if (perc_cfg_.dim == drone_planner::PlannerDim::THREE_D) {
      // ============ 3D A* ============
      engine_->config() = perc_cfg_;
      engine_->rebuild(obstacles, inflate);

      // 起点保护
      {
        Vec3i sg = engine_->voxelGrid().worldToGrid(drone_pos);
        for (int dz = -1; dz <= 1; ++dz)
          for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx)
              if (engine_->voxelGrid().inBounds(sg.x()+dx, sg.y()+dy, sg.z()+dz))
                const_cast<drone_common::VoxelGrid&>(engine_->voxelGrid())
                    .at(sg.x()+dx, sg.y()+dy, sg.z()+dz) = 0;
      }

      const auto& grid3d = engine_->activeGrid3D(drone_pos, drone_fwd);
      Vec3d goal3d(gx, gy, z_cruise);

      auto path3d = drone_planner::astarSearch3D(grid3d, drone_pos, goal3d);
      if (path3d.empty()) { publishStatus("NO_PATH"); return; }

      // 3D 路径 → 发布 planned_path (z 随航点变化)
      publishPath3D(path3d);

      // safe_goal: 沿 3D 路径单调推进
      double best_dist = 1e9; int closest_i = 0;
      for (int i = 0; i < static_cast<int>(path3d.size()); ++i) {
        double d = (path3d[i] - drone_pos).norm();
        if (d < best_dist) { best_dist = d; closest_i = i; }
      }
      int n = static_cast<int>(path3d.size()) - 1;
      if (closest_i >= n) { last_look_i_ = n; }
      else {
        int target = std::min(closest_i + 1, n);
        int advance = std::min(last_look_i_ + 1, target);
        if (advance > last_look_i_) last_look_i_ = advance;
        if (last_look_i_ > n) last_look_i_ = n;
      }
      publishSafeGoal(path3d[last_look_i_].x(), path3d[last_look_i_].y(), path3d[last_look_i_].z());

    } else {
      // ============ 2D A* ============
      auto grid2d = engine_->buildGrid2D(
          obstacles, resolution_, bounds_[0], bounds_[1], bounds_[2], bounds_[3],
          inflate, z_cruise, drone_pos, drone_fwd);

      // 起点保护
      int psx, psy;
      grid2d.worldToGrid(px, py, psx, psy);
      int clear_r = static_cast<int>(std::ceil((safety_dist_ + drone_r_) / resolution_));
      for (int dy = -clear_r; dy <= clear_r; ++dy)
        for (int dx = -clear_r; dx <= clear_r; ++dx)
          if (grid2d.inBounds(psx + dx, psy + dy))
            grid2d.at(psx + dx, psy + dy) = 0;

      auto path = drone_planner::astarSearch(grid2d, px, py, gx, gy);
      if (path.empty()) { publishStatus("NO_PATH"); return; }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 3000,
          "planned_path: %zu waypoints (smoothed)", path.size());

      publishPath(path, z_cruise);

      double best_dist = 1e9; int closest_i = 0;
      for (int i = 0; i < static_cast<int>(path.size()); ++i) {
        double d = std::hypot(path[i].x() - px, path[i].y() - py);
        if (d < best_dist) { best_dist = d; closest_i = i; }
      }
      int n = static_cast<int>(path.size()) - 1;
      if (closest_i >= n) { last_look_i_ = n; }
      else {
        int target = std::min(closest_i + 1, n);
        int advance = std::min(last_look_i_ + 1, target);
        if (advance > last_look_i_) last_look_i_ = advance;
        if (last_look_i_ > n) last_look_i_ = n;
      }
      publishSafeGoal(path[last_look_i_].x(), path[last_look_i_].y(), z_cruise);
    }

    // FOV 可视化
    publishFOVVisual(px, py, pz, drone_fwd);

    if (last_look_i_ < static_cast<int>(last_path_size_) - 1) {
      publishStatus("OK");
    } else {
      publishStatus("AT_GOAL");
    }
  }

  // ---- FOV 锥体 + 体素可视化 ----
  void publishFOVVisual(double px, double py, double pz, const Vec3d& drone_fwd) {
    if (perc_cfg_.mode != drone_planner::PerceptionMode::FOV) {
      // 全局模式 → 清空 FOV 可视化
      visualization_msgs::msg::Marker del;
      del.action = visualization_msgs::msg::Marker::DELETE;
      del.ns = "fov_cone"; del.id = 0; fov_cone_pub_->publish(del);
      visualization_msgs::msg::MarkerArray del_arr;
      visualization_msgs::msg::Marker del2;
      del2.action = visualization_msgs::msg::Marker::DELETEALL;
      del2.ns = "fov_voxels";
      del_arr.markers.push_back(del2);
      fov_voxels_pub_->publish(del_arr);
      return;
    }

    auto fov = engine_->queryFOV(Vec3d(px, py, pz), drone_fwd);

    // 锥体 (TRIANGLE_LIST — 简化为金字塔四条边)
    {
      visualization_msgs::msg::Marker cone;
      cone.header.stamp = now(); cone.header.frame_id = grid_frame_;
      cone.ns = "fov_cone"; cone.id = 0;
      cone.type = visualization_msgs::msg::Marker::LINE_LIST;
      cone.action = visualization_msgs::msg::Marker::ADD;
      cone.color.r = 0.0f; cone.color.g = 1.0f; cone.color.b = 1.0f; cone.color.a = 0.3f;
      cone.scale.x = 0.03;  // line width

      double half_h = perc_cfg_.fov_h_rad * 0.5;
      double half_v = perc_cfg_.fov_v_rad * 0.5;
      double rng = perc_cfg_.fov_range;

      // 四个角点
      Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Vec3d(1, 0, 0), drone_fwd);
      auto rot = [&](double ha, double va) -> Vec3d {
        Vec3d v(rng, rng * std::tan(ha), rng * std::tan(va));
        return Vec3d(px, py, pz) + q.toRotationMatrix() * v;
      };
      Vec3d apex(px, py, pz);
      Vec3d tl = rot(half_h, half_v);
      Vec3d tr = rot(-half_h, half_v);
      Vec3d bl = rot(half_h, -half_v);
      Vec3d br = rot(-half_h, -half_v);

      auto add_l = [&](const Vec3d& a, const Vec3d& b) {
        geometry_msgs::msg::Point pa, pb;
        pa.x=a.x(); pa.y=a.y(); pa.z=a.z();
        pb.x=b.x(); pb.y=b.y(); pb.z=b.z();
        cone.points.push_back(pa); cone.points.push_back(pb);
      };
      add_l(apex, tl); add_l(apex, tr); add_l(apex, bl); add_l(apex, br);
      add_l(tl, tr); add_l(tr, br); add_l(br, bl); add_l(bl, tl);

      fov_cone_pub_->publish(cone);
    }

    // 可见体素 (CUBE_LIST — 绿色半透明，稀疏发布 0.5Hz)
    static int voxel_frame_cnt = 0;
    if (++voxel_frame_cnt % 2 != 0) return;  // 1Hz → 0.5Hz

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker vm;
    vm.header.stamp = now(); vm.header.frame_id = grid_frame_;
    vm.ns = "fov_voxels"; vm.id = 0;
    vm.type = visualization_msgs::msg::Marker::CUBE_LIST;
    vm.action = visualization_msgs::msg::Marker::ADD;
    vm.color.r = 0.0f; vm.color.g = 1.0f; vm.color.b = 0.0f; vm.color.a = 0.4f;
    vm.scale.x = vm.scale.y = vm.scale.z = perc_cfg_.voxel_res * 0.9;

    size_t cnt = 0;
    for (int z = 0; z < fov.nz && cnt < 5000; ++z) {
      for (int y = 0; y < fov.ny && cnt < 5000; ++y) {
        for (int x = 0; x < fov.nx && cnt < 5000; ++x) {
          size_t idx = x + fov.nx * (y + fov.ny * z);
          if (fov.visible[idx] && engine_->voxelGrid().at(x, y, z) == 1) {
            Vec3d w = engine_->voxelGrid().gridToWorld(Vec3i(x, y, z));
            geometry_msgs::msg::Point pt;
            pt.x = w.x(); pt.y = w.y(); pt.z = w.z();
            vm.points.push_back(pt);
            ++cnt;
          }
        }
      }
    }
    arr.markers.push_back(vm);
    fov_voxels_pub_->publish(arr);
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
    last_path_size_ = path.size();
  }

  void publishPath3D(const std::vector<Vec3d>& path) {
    nav_msgs::msg::Path msg;
    msg.header.stamp = now(); msg.header.frame_id = grid_frame_;
    for (const auto& wp : path) {
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = grid_frame_;
      ps.pose.position.x = wp.x(); ps.pose.position.y = wp.y(); ps.pose.position.z = wp.z();
      ps.pose.orientation.w = 1.0;
      msg.poses.push_back(ps);
    }
    path_pub_->publish(msg);
    last_path_size_ = path.size();
  }

  // 参数
  double resolution_, safety_dist_, drone_r_, cruise_z_, lookahead_, advance_tol_;
  bool use_goal_z_;
  std::vector<double> bounds_;
  std::string grid_frame_;
  drone_planner::PerceptionConfig perc_cfg_;
  int last_look_i_ = 0;
  int last_path_size_ = 0;

  std::unique_ptr<drone_planner::PerceptionEngine> engine_;

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
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr fov_cone_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr fov_voxels_pub_;
  rclcpp::TimerBase::SharedPtr replan_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}

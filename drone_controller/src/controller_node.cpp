// controller_node.cpp — 级联 PD 位置控制器 ROS2 节点
//
// 订阅 /drone/odom (Odometry) + /drone/safe_goal (PoseStamped)
//   （planner 禁用时，launch remap safe_goal → /drone/goal）
// 200 Hz 控制律，发布 /drone/motor_rpm_cmd (Float32MultiArray [FL,FR,BL,BR] RPM)

#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "drone_common/drone_common.hpp"
#include "drone_controller/controller.hpp"

using drone_controller::DroneController;
using drone_controller::Params;
using drone_common::Vec3d;
using drone_common::Quatd;

class ControllerNode : public rclcpp::Node {
 public:
  ControllerNode() : Node("drone_controller") {
    declareParams();
    ctrl_ = std::make_unique<DroneController>(loadParams());
    ctrl_dt_ = get_parameter("ctrl_dt").as_double();

    // 订阅 odom
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/drone/odom", rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          last_odom_ = m;
        });

    // 订阅 safe_goal（planner 禁用时 launch remap 到 /drone/goal）
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/drone/safe_goal", 10,
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr m) {
          std::lock_guard<std::mutex> lk(mtx_);
          last_goal_ = m;
        });

    // 发布 RPM
    rpm_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
        "/drone/motor_rpm_cmd", 10);

    // 200 Hz 控制定时器
    int period_us = static_cast<int>(ctrl_dt_ * 1e6);
    ctrl_timer_ = create_wall_timer(
        std::chrono::microseconds(period_us), [this]() { tick(); });

    RCLCPP_INFO(get_logger(),
        "controller ready: mode=%s Kp=[%.1f %.1f %.1f] Kd=[%.1f %.1f %.1f] dt=%.2fms",
        ctrl_->params().control_mode.c_str(),
        ctrl_->params().Kp_pos.x(), ctrl_->params().Kp_pos.y(), ctrl_->params().Kp_pos.z(),
        ctrl_->params().Kd_pos.x(), ctrl_->params().Kd_pos.y(), ctrl_->params().Kd_pos.z(),
        ctrl_dt_ * 1e3);

    // 运行时参数变更回调（支持 ros2 param set 热切换 LADRC 参数）
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          return onParamChange(params);
        });
  }

 private:
  void declareParams() {
    declare_parameter<double>("mass", 1.0);
    declare_parameter<double>("arm_length", 0.2);
    declare_parameter<double>("k_F", 1.0);
    declare_parameter<double>("k_M", 0.05);
    declare_parameter<std::vector<double>>("Kp_pos", {2.0, 2.0, 3.0});
    declare_parameter<std::vector<double>>("Kd_pos", {2.0, 2.0, 2.4});
    declare_parameter<std::vector<double>>("Ki_pos", {0.0, 0.0, 0.0});
    declare_parameter<double>("Ki_max", 1.0);
    declare_parameter<double>("a_xy_max", 4.0);
    declare_parameter<double>("a_z_min", -3.0);
    declare_parameter<double>("a_z_max", 6.0);
    declare_parameter<double>("a_max_vec", 8.0);
    declare_parameter<double>("d_far", 10.0);
    declare_parameter<std::vector<double>>("Kp_att", {8.0, 8.0, 3.0});
    declare_parameter<std::vector<double>>("Kd_rate", {0.8, 0.8, 0.7});
    // LADRC 参数
    declare_parameter<std::string>("control_mode", "pd");
    declare_parameter<std::vector<double>>("ladrc_b0", {1.0, 1.0, 1.0});
    declare_parameter<std::vector<double>>("ladrc_wc", {1.5, 1.5, 2.0});
    declare_parameter<std::vector<double>>("ladrc_wo", {6.0, 6.0, 10.0});
    declare_parameter<double>("F_min", 0.0);
    declare_parameter<double>("F_max", 39.24);
    declare_parameter<double>("tau_max", 5.0);
    declare_parameter<double>("omega_max", 1000.0);
    declare_parameter<double>("rpm_min", 0.0);
    declare_parameter<double>("rpm_max", 10000.0);
    declare_parameter<double>("ctrl_dt", 0.005);
  }

  Params loadParams() {
    Params p;
    p.mass = get_parameter("mass").as_double();
    p.arm_length = get_parameter("arm_length").as_double();
    p.k_F = get_parameter("k_F").as_double();
    p.k_M = get_parameter("k_M").as_double();
    auto kp = get_parameter("Kp_pos").as_double_array();
    auto kd = get_parameter("Kd_pos").as_double_array();
    auto ki = get_parameter("Ki_pos").as_double_array();
    auto kpa = get_parameter("Kp_att").as_double_array();
    auto kdr = get_parameter("Kd_rate").as_double_array();
    if (kp.size() == 3) p.Kp_pos << kp[0], kp[1], kp[2];
    if (kd.size() == 3) p.Kd_pos << kd[0], kd[1], kd[2];
    if (ki.size() == 3) p.Ki_pos << ki[0], ki[1], ki[2];
    if (kpa.size() == 3) p.Kp_att << kpa[0], kpa[1], kpa[2];
    if (kdr.size() == 3) p.Kd_rate << kdr[0], kdr[1], kdr[2];
    p.Ki_max = get_parameter("Ki_max").as_double();
    p.a_xy_max = get_parameter("a_xy_max").as_double();
    p.a_z_min = get_parameter("a_z_min").as_double();
    p.a_z_max = get_parameter("a_z_max").as_double();
    p.a_max_vec = get_parameter("a_max_vec").as_double();
    p.d_far = get_parameter("d_far").as_double();
    p.F_min = get_parameter("F_min").as_double();
    p.F_max = get_parameter("F_max").as_double();
    p.tau_max = get_parameter("tau_max").as_double();
    p.omega_max = get_parameter("omega_max").as_double();
    p.rpm_min = get_parameter("rpm_min").as_double();
    p.rpm_max = get_parameter("rpm_max").as_double();
    p.ctrl_dt = get_parameter("ctrl_dt").as_double();
    // LADRC
    p.control_mode = get_parameter("control_mode").as_string();
    auto b0 = get_parameter("ladrc_b0").as_double_array();
    auto wc = get_parameter("ladrc_wc").as_double_array();
    auto wo = get_parameter("ladrc_wo").as_double_array();
    if (b0.size()>=3) p.ladrc_b0 << b0[0],b0[1],b0[2];
    if (wc.size()>=3) p.ladrc_wc << wc[0],wc[1],wc[2];
    if (wo.size()>=3) p.ladrc_wo << wo[0],wo[1],wo[2];
    return p;
  }

  rcl_interfaces::msg::SetParametersResult onParamChange(
      const std::vector<rclcpp::Parameter>& params) {
    for (const auto& p : params) {
      if (p.get_name() == "control_mode") {
        ctrl_->setControlMode(p.as_string());
        RCLCPP_INFO(get_logger(), "switched to mode=%s", p.as_string().c_str());
      } else if (p.get_name() == "ladrc_b0") {
        auto b0 = get_parameter("ladrc_b0").as_double_array();
        ctrl_->setLADRCParams(
            Vec3d(b0[0],b0[1],b0[2]),
            ctrl_->params().ladrc_wc,
            ctrl_->params().ladrc_wo);
        RCLCPP_INFO(get_logger(), "LADRC b0 updated: [%.3f %.3f %.3f]", b0[0],b0[1],b0[2]);
      } else if (p.get_name() == "ladrc_wc") {
        auto wc = get_parameter("ladrc_wc").as_double_array();
        ctrl_->setLADRCParams(
            ctrl_->params().ladrc_b0,
            Vec3d(wc[0],wc[1],wc[2]),
            ctrl_->params().ladrc_wo);
        RCLCPP_INFO(get_logger(), "LADRC wc updated: [%.1f %.1f %.1f]", wc[0],wc[1],wc[2]);
      } else if (p.get_name() == "ladrc_wo") {
        auto wo = get_parameter("ladrc_wo").as_double_array();
        ctrl_->setLADRCParams(
            ctrl_->params().ladrc_b0,
            ctrl_->params().ladrc_wc,
            Vec3d(wo[0],wo[1],wo[2]));
        RCLCPP_INFO(get_logger(), "LADRC wo updated: [%.1f %.1f %.1f]", wo[0],wo[1],wo[2]);
      } else if (p.get_name() == "Kp_pos" || p.get_name() == "Kd_pos" ||
                 p.get_name() == "Kp_att" || p.get_name() == "Kd_rate") {
        auto kp = get_parameter("Kp_pos").as_double_array();
        auto kd = get_parameter("Kd_pos").as_double_array();
        auto kpa = get_parameter("Kp_att").as_double_array();
        auto kdr = get_parameter("Kd_rate").as_double_array();
        ctrl_->setPDGains(
            Vec3d(kp[0],kp[1],kp[2]), Vec3d(kd[0],kd[1],kd[2]),
            Vec3d(kpa[0],kpa[1],kpa[2]), Vec3d(kdr[0],kdr[1],kdr[2]));
        RCLCPP_INFO(get_logger(), "PD gains updated: Kp=[%.1f %.1f %.1f] Kd=[%.1f %.1f %.1f]",
            kp[0],kp[1],kp[2], kd[0],kd[1],kd[2]);
      }
    }
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }

  void tick() {
    nav_msgs::msg::Odometry::SharedPtr odom;
    geometry_msgs::msg::PoseStamped::SharedPtr goal;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      odom = last_odom_;
      goal = last_goal_;
    }
    if (!odom) return;  // 还没收到 odom

    // 提取状态
    Vec3d p(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Vec3d v(odom->twist.twist.linear.x, odom->twist.twist.linear.y, odom->twist.twist.linear.z);
    // twist 在 base_link，需要转世界系
    {
      Quatd q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
              odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
      v = q.toRotationMatrix() * v;  // 机体系→世界系
    }
    Quatd q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    Vec3d w(odom->twist.twist.angular.x, odom->twist.twist.angular.y, odom->twist.twist.angular.z);

    // 目标 — 默认悬停在当前位置 1.5 m
    Vec3d goal_p = p; goal_p.z() = 1.5;
    double goal_yaw = 0.0;
    if (goal) {
      goal_p.x() = goal->pose.position.x;
      goal_p.y() = goal->pose.position.y;
      goal_p.z() = goal->pose.position.z;
      // 从 goal 四元数取 yaw
      Quatd qg(goal->pose.orientation.w, goal->pose.orientation.x,
               goal->pose.orientation.y, goal->pose.orientation.z);
      auto euler = qg.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX
      goal_yaw = euler(0);
      // 2D Goal Point 给 z=0 → 抬到最低 1.0
      if (goal_p.z() < 0.5) goal_p.z() = 1.5;
    }

    auto rpm = ctrl_->step(p, v, q, w, goal_p, goal_yaw);
    // 限幅
    for (int i = 0; i < 4; ++i) {
      rpm[i] = std::clamp(rpm[i], ctrl_->params().rpm_min, ctrl_->params().rpm_max);
    }

    auto msg = std_msgs::msg::Float32MultiArray();
    msg.data = {rpm[0], rpm[1], rpm[2], rpm[3]};
    rpm_pub_->publish(msg);

    ++tick_cnt_;
    if (tick_cnt_ % 200 == 0) {  // ~1 Hz
      RCLCPP_INFO(get_logger(),
          "pos=[%.2f %.2f %.2f] goal=[%.2f %.2f %.2f] err=%.3f rpm=[%.0f %.0f %.0f %.0f]",
          p.x(), p.y(), p.z(), goal_p.x(), goal_p.y(), goal_p.z(),
          (goal_p - p).norm(), rpm[0], rpm[1], rpm[2], rpm[3]);
    }
  }

  std::unique_ptr<DroneController> ctrl_;
  double ctrl_dt_ = 0.005;
  uint64_t tick_cnt_ = 0;

  std::mutex mtx_;
  nav_msgs::msg::Odometry::SharedPtr last_odom_;
  geometry_msgs::msg::PoseStamped::SharedPtr last_goal_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_pub_;
  rclcpp::TimerBase::SharedPtr ctrl_timer_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControllerNode>());
  rclcpp::shutdown();
  return 0;
}
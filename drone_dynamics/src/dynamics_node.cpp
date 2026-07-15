// dynamics_node.cpp — 四旋翼动力学 ROS2 节点
// MT executor(2): dyn_group_ 隔离 1kHz timer, 默认组处理 RPM 订阅 + 发布

#include <memory>
#include <array>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "drone_dynamics/dynamics.hpp"
#include "drone_common/noise.hpp"

using drone_common::Vec3d;
using drone_common::Mat3d;
using drone_common::ImuNoiseModel;
using drone_dynamics::DroneDynamics;
using drone_dynamics::Params;
using namespace std::chrono_literals;

static constexpr double PI = 3.14159265358979323846;

// RPM -> rad/s
static inline double rpm2omega(double rpm) { return rpm * 2.0 * PI / 60.0; }

class DroneDynamicsNode : public rclcpp::Node {
public:
  DroneDynamicsNode() : Node("drone_dynamics") {
    declare_parameter<double>("mass", 1.0);
    declare_parameter<std::vector<double>>("inertia", {0.02, 0.02, 0.04});
    declare_parameter<double>("k_F", 1.0);
    declare_parameter<double>("k_M", 0.05);
    declare_parameter<double>("arm_length", 0.2);
    declare_parameter<double>("motor_tau", 0.02);
    declare_parameter<double>("omega_min", 0.0);
    declare_parameter<double>("omega_max", 1.0e3);
    declare_parameter<double>("sim_dt", 0.001);
    declare_parameter<bool>("add_linear_drag", false);
    declare_parameter<double>("drag_coeff", 0.1);
    declare_parameter<bool>("wind_enabled", false);
    declare_parameter<std::vector<double>>("wind_force", {0.0, 0.0, 0.0});
    declare_parameter<double>("wind_gust_amplitude", 0.0);
    declare_parameter<double>("wind_gust_period", 2.0);
    declare_parameter<bool>("imu_noise_enabled", false);
    declare_parameter<double>("accel_noise_density", 0.0);
    declare_parameter<double>("accel_bias_init", 0.0);
    declare_parameter<double>("accel_bias_rw", 0.0);
    declare_parameter<double>("gyro_noise_density", 0.0);
    declare_parameter<double>("gyro_bias_init", 0.0);
    declare_parameter<double>("gyro_bias_rw", 0.0);
    declare_parameter<double>("odom_pos_noise", 0.0);
    declare_parameter<double>("odom_vel_noise", 0.0);
    declare_parameter<int64_t>("noise_seed", 12345);
    declare_parameter<std::vector<double>>("init_pose", {0.0,0.0,0.0,0.0});
    declare_parameter<std::string>("frame_id", "map");

    Params p = loadParams();
    dynamics_ = std::make_unique<DroneDynamics>(p);
    sim_dt_ = p.sim_dt;
    frame_id_ = get_parameter("frame_id").as_string();

    // 噪声模型初始化
    noise_seed_ = static_cast<uint64_t>(get_parameter("noise_seed").as_int());
    imu_noise_enabled_ = get_parameter("imu_noise_enabled").as_bool();
    accel_nd_ = get_parameter("accel_noise_density").as_double();
    accel_bi_ = get_parameter("accel_bias_init").as_double();
    accel_brw_ = get_parameter("accel_bias_rw").as_double();
    gyro_nd_ = get_parameter("gyro_noise_density").as_double();
    gyro_bi_ = get_parameter("gyro_bias_init").as_double();
    gyro_brw_ = get_parameter("gyro_bias_rw").as_double();
    odom_pos_noise_ = get_parameter("odom_pos_noise").as_double();
    odom_vel_noise_ = get_parameter("odom_vel_noise").as_double();
    if (imu_noise_enabled_) {
      imu_noise_ = std::make_unique<ImuNoiseModel>(
          accel_nd_, accel_bi_, accel_brw_, gyro_nd_, gyro_bi_, gyro_brw_, noise_seed_);
    }
    odom_rng_.seed(noise_seed_ + 1);

    // RPM 订阅 — 独立 MutuallyExclusive group (不被 1kHz timer 饿死)
    auto sub_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = sub_group;
    rpm_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "/drone/motor_rpm_cmd", rclcpp::SensorDataQoS(),
        [this](const std_msgs::msg::Float32MultiArray::SharedPtr m){
          std::lock_guard<std::mutex> lk(rpm_mutex_);
          if (m->data.size() >= 4)
            for (int i=0;i<4;++i) last_rpm_[i] = m->data[i];
        }, so);

    // 发布 (默认 group)
    odom_pub_   = create_publisher<nav_msgs::msg::Odometry>("/drone/odom", 10);
    imu_pub_    = create_publisher<sensor_msgs::msg::Imu>("/drone/imu", 10);
    path_pub_   = create_publisher<nav_msgs::msg::Path>("/drone/path", 10);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/drone/drone_marker", 10);
    tf_br_      = std::make_unique<tf2_ros::TransformBroadcaster>(this);
    static_tf_  = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

    // static tf base_link->imu_link
    geometry_msgs::msg::TransformStamped stf;
    stf.header.stamp = now(); stf.header.frame_id = "base_link"; stf.child_frame_id = "imu_link";
    stf.transform.rotation.w = 1.0;
    static_tf_->sendTransform(stf);

    // 1kHz dynamics timer — 独立 group
    dyn_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    dyn_timer_ = create_wall_timer(
        std::chrono::microseconds(static_cast<int>(sim_dt_*1e6)),
        [this]{ tick(); }, dyn_group_);

    pub_timer_   = create_wall_timer(10ms,  [this]{ pubAll(); });
    path_timer_  = create_wall_timer(100ms, [this]{ pubPath(); });
    mark_timer_  = create_wall_timer(100ms, [this]{ pubMarker(); });

    path_msg_.header.frame_id = frame_id_;

    RCLCPP_INFO(get_logger(), "ready mass=%.2f kF=%.2f l=%.3f dt=%.2fms",
                p.mass, p.k_F, p.arm_length, sim_dt_*1e3);

    // 运行时参数变更回调 — 支持 ros2 param set 热更新 wind/noise
    param_cb_handle_ = add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
          rcl_interfaces::msg::SetParametersResult result; result.successful = true;
          for (const auto& p : params) {
            const auto& n = p.get_name();
            if (n == "wind_enabled") {
              dynamics_->setWindEnabled(p.as_bool());
              RCLCPP_INFO(get_logger(), "wind_enabled=%d", p.as_bool());
            } else if (n == "wind_force") {
              auto v = p.as_double_array();
              if (v.size() >= 3) { dynamics_->setWindForce(Vec3d(v[0],v[1],v[2])); }
            } else if (n == "wind_gust_amplitude" || n == "wind_gust_period") {
              dynamics_->setWindGust(
                  get_parameter("wind_gust_amplitude").as_double(),
                  get_parameter("wind_gust_period").as_double());
            } else if (n == "imu_noise_enabled" || n.rfind("accel_",0)==0 || n.rfind("gyro_",0)==0
                       || n == "odom_pos_noise" || n == "odom_vel_noise") {
              // Re-create noise model with new params
              imu_noise_enabled_ = get_parameter("imu_noise_enabled").as_bool();
              accel_nd_ = get_parameter("accel_noise_density").as_double();
              accel_bi_ = get_parameter("accel_bias_init").as_double();
              accel_brw_ = get_parameter("accel_bias_rw").as_double();
              gyro_nd_ = get_parameter("gyro_noise_density").as_double();
              gyro_bi_ = get_parameter("gyro_bias_init").as_double();
              gyro_brw_ = get_parameter("gyro_bias_rw").as_double();
              odom_pos_noise_ = get_parameter("odom_pos_noise").as_double();
              odom_vel_noise_ = get_parameter("odom_vel_noise").as_double();
              if (imu_noise_enabled_) {
                imu_noise_ = std::make_unique<ImuNoiseModel>(
                    accel_nd_, accel_bi_, accel_brw_, gyro_nd_, gyro_bi_, gyro_brw_, noise_seed_);
              } else { imu_noise_.reset(); }
              RCLCPP_INFO(get_logger(), "noise updated: imu=%d odom_pos=%.3f odom_vel=%.3f",
                  imu_noise_enabled_, odom_pos_noise_, odom_vel_noise_);
            }
          }
          return result;
        });
  }

private:
  Params loadParams(){
    Params p;
    p.mass=get_parameter("mass").as_double();
    auto I=get_parameter("inertia").as_double_array();
    if(I.size()!=3) throw std::runtime_error("inertia must be 3");
    p.inertia<<I[0],I[1],I[2];
    p.k_F=get_parameter("k_F").as_double();
    p.k_M=get_parameter("k_M").as_double();
    p.arm_length=get_parameter("arm_length").as_double();
    p.motor_tau=get_parameter("motor_tau").as_double();
    p.omega_min=get_parameter("omega_min").as_double();
    p.omega_max=get_parameter("omega_max").as_double();
    p.sim_dt=get_parameter("sim_dt").as_double();
    p.add_linear_drag=get_parameter("add_linear_drag").as_bool();
    p.drag_coeff=get_parameter("drag_coeff").as_double();
    p.wind_enabled=get_parameter("wind_enabled").as_bool();
    auto wf=get_parameter("wind_force").as_double_array();
    if(wf.size()>=3) p.wind_force<<wf[0],wf[1],wf[2];
    p.wind_gust_amplitude=get_parameter("wind_gust_amplitude").as_double();
    p.wind_gust_period=get_parameter("wind_gust_period").as_double();
    auto init=get_parameter("init_pose").as_double_array();
    if(init.size()>=4){ p.init_pos<<init[0],init[1],init[2]; p.init_yaw=init[3]; }
    return p;
  }

  void tick(){
    std::array<double,4> rpm;
    { std::lock_guard<std::mutex> lk(rpm_mutex_); rpm=last_rpm_; }
    dynamics_->stepFromRpmCmd(rpm, sim_dt_);
    ++tick_cnt_;
    if(tick_cnt_%1000==0){
      const auto& s=dynamics_->state();
      RCLCPP_INFO(get_logger(),
        "t=%ds p=[%.2f %.2f %.2f] v=[%.3f %.3f %.3f] w=[%.3f %.3f %.3f %.3f] rpm=[%.1f %.1f %.1f %.1f]",
        tick_cnt_/1000, s.p.x(),s.p.y(),s.p.z(), s.v.x(),s.v.y(),s.v.z(),
        s.motor_omega[0],s.motor_omega[1],s.motor_omega[2],s.motor_omega[3],
        rpm[0],rpm[1],rpm[2],rpm[3]);
    }
  }

  void pubAll(){
    const auto& s=dynamics_->state(); auto ts=now();
    Vec3d true_pos(s.p.x(), s.p.y(), s.p.z());
    Vec3d R_vb = s.q.toRotationMatrix().transpose() * s.v;
    Vec3d true_vel_body(R_vb.x(), R_vb.y(), R_vb.z());
    Vec3d true_w_body(s.w_body.x(), s.w_body.y(), s.w_body.z());
    Mat3d RBW = s.q.toRotationMatrix().transpose();
    Vec3d az(0, 0, 9.80665);
    Vec3d true_accel_body = RBW * (s.a_world_last + az);

    // 传感器噪声
    Vec3d pos_out = true_pos;
    Vec3d vel_out = true_vel_body;
    Vec3d accel_out = true_accel_body;
    Vec3d gyro_out = true_w_body;
    if (imu_noise_enabled_ && imu_noise_) {
      accel_out = imu_noise_->corruptAccel(true_accel_body, sim_dt_);
      gyro_out = imu_noise_->corruptGyro(true_w_body, sim_dt_);
    }
    if (odom_pos_noise_ > 0)
      pos_out = drone_common::addPositionNoise(true_pos, odom_pos_noise_, odom_rng_);
    if (odom_vel_noise_ > 0)
      vel_out = drone_common::addVelocityNoise(true_vel_body, odom_vel_noise_, odom_rng_);

    nav_msgs::msg::Odometry o;
    o.header.stamp=ts; o.header.frame_id=frame_id_; o.child_frame_id="base_link";
    o.pose.pose.position.x=pos_out.x(); o.pose.pose.position.y=pos_out.y(); o.pose.pose.position.z=pos_out.z();
    o.pose.pose.orientation.x=s.q.x(); o.pose.pose.orientation.y=s.q.y();
    o.pose.pose.orientation.z=s.q.z(); o.pose.pose.orientation.w=s.q.w();
    o.twist.twist.linear.x=vel_out.x(); o.twist.twist.linear.y=vel_out.y(); o.twist.twist.linear.z=vel_out.z();
    o.twist.twist.angular.x=gyro_out.x(); o.twist.twist.angular.y=gyro_out.y(); o.twist.twist.angular.z=gyro_out.z();
    odom_pub_->publish(o);

    sensor_msgs::msg::Imu imu;
    imu.header.stamp=ts; imu.header.frame_id="imu_link";
    imu.orientation=o.pose.pose.orientation;
    imu.angular_velocity.x=gyro_out.x(); imu.angular_velocity.y=gyro_out.y(); imu.angular_velocity.z=gyro_out.z();
    imu.linear_acceleration.x=accel_out.x(); imu.linear_acceleration.y=accel_out.y(); imu.linear_acceleration.z=accel_out.z();
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp=ts; tf.header.frame_id=frame_id_; tf.child_frame_id="base_link";
    // tf 永远用真实位姿（不受噪声影响）
    tf.transform.translation.x=s.p.x(); tf.transform.translation.y=s.p.y(); tf.transform.translation.z=s.p.z();
    tf.transform.rotation=o.pose.pose.orientation;
    tf_br_->sendTransform(tf);
  }

  void pubPath(){
    const auto& s=dynamics_->state();
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp=now(); ps.header.frame_id=frame_id_;
    ps.pose.position.x=s.p.x(); ps.pose.position.y=s.p.y(); ps.pose.position.z=s.p.z();
    ps.pose.orientation.x=s.q.x(); ps.pose.orientation.y=s.q.y(); ps.pose.orientation.z=s.q.z(); ps.pose.orientation.w=s.q.w();
    path_msg_.poses.push_back(ps);
    if(path_msg_.poses.size()>2000) path_msg_.poses.erase(path_msg_.poses.begin());
    path_msg_.header.stamp=ps.header.stamp;
    path_pub_->publish(path_msg_);
  }

  void pubMarker(){
    const auto& s=dynamics_->state();
    visualization_msgs::msg::Marker m;
    m.header.stamp=now(); m.header.frame_id=frame_id_; m.ns="drone"; m.id=0; m.type=m.ARROW; m.action=m.ADD;
    m.pose.position.x=s.p.x(); m.pose.position.y=s.p.y(); m.pose.position.z=s.p.z();
    m.pose.orientation.x=s.q.x(); m.pose.orientation.y=s.q.y(); m.pose.orientation.z=s.q.z(); m.pose.orientation.w=s.q.w();
    m.scale.x=0.4; m.scale.y=0.08; m.scale.z=0.08;
    m.color.r=0.2; m.color.g=0.8; m.color.b=0.2; m.color.a=0.9;
    visualization_msgs::msg::MarkerArray arr; arr.markers.push_back(m);
    marker_pub_->publish(arr);
  }

  std::unique_ptr<DroneDynamics> dynamics_;
  double sim_dt_=0.001; std::string frame_id_="map";
  uint64_t tick_cnt_=0;
  std::mutex rpm_mutex_; std::array<double,4> last_rpm_{0,0,0,0};

  // 传感器噪声
  std::unique_ptr<ImuNoiseModel> imu_noise_;
  std::mt19937_64 odom_rng_{12346};
  bool imu_noise_enabled_ = false;
  double accel_nd_, accel_bi_, accel_brw_, gyro_nd_, gyro_bi_, gyro_brw_;
  double odom_pos_noise_ = 0.0, odom_vel_noise_ = 0.0;
  uint64_t noise_seed_ = 12345;

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr rpm_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_br_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_;
  nav_msgs::msg::Path path_msg_;

  rclcpp::CallbackGroup::SharedPtr dyn_group_;
  rclcpp::TimerBase::SharedPtr dyn_timer_, pub_timer_, path_timer_, mark_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  auto node=std::make_shared<DroneDynamicsNode>();
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 2);
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
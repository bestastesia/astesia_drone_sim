#pragma once
//
// controller.hpp — 级联 PD 位置控制器 + mixer 逆（纯 C++，不依赖 ROS）
// ----------------------------------------------------------------------
// 输入：当前 odom (p, v, q, w_body) + 目标 PoseStamped (p_goal, yaw_goal)
// 输出：4 个期望 RPM
// 控制律：
//   1. 位置 PD → 期望世界加速度 a_des
//   2. 由 a_des 计算期望推力 + 机体系 z_B_des
//   3. z_B_des + yaw_goal → 期望旋转矩阵 R_des（避免 Euler 奇异）
//   4. 几何姿态误差（Lee）→ 3 轴力矩 τ_des
//   5. B⁻¹[F, τx, τy, τz] → 4 个 ω² → clamp → RPM
#pragma once

#include <Eigen/Dense>
#include <array>
#include <algorithm>
#include <cassert>
#include <cmath>

#include "drone_common/drone_common.hpp"
#include "drone_controller/ladrc.hpp"

namespace drone_controller {

using drone_common::Vec3d;
using drone_common::Quatd;
using drone_common::Mat3d;
using drone_common::Wrench;
using drone_common::MotorSq;

struct Params {
  // 位置环 PD（世界系每轴独立）
  Vec3d Kp_pos{3.0, 3.0, 5.0};   // P
  Vec3d Kd_pos{1.5, 1.5, 2.5};   // D
  Vec3d Ki_pos{0, 0, 0};         // I（默认禁用，加才有防饱和）
  double Ki_max = 1.0;            // I 上限
  // 加速度限幅
  double a_xy_max = 5.0;          // 水平 m/s²
  double a_z_min = -3.0;          // 下降
  double a_z_max = 8.0;           // 上升
  double a_max_vec = 10.0;        // 向量模上限
  // 远目标安全钳
  double d_far = 10.0;            // 距离阈值 m
  // 姿态 / 角速度环
  Vec3d Kp_att{10.0, 10.0, 5.0}; // attitude → torque
  Vec3d Kd_rate{3.0, 3.0, 2.0};  // rate → torque
  // 限幅
  double F_min = 0.0;             // 最小推力 N
  double F_max = 39.24;           // 最大推力 (4×kF×ω²_max)
  double tau_max = 5.0;           // 单轴力矩上限 N·m
  double omega_max = 1000.0;      // rad/s
  double rpm_min = 0.0;
  double rpm_max = 10000.0;
  // 物理参数（与 dynamics 保持一致）
  double mass = 1.0;
  double arm_length = 0.2;
  double k_F = 1.0;
  double k_M = 0.05;
  // 控制周期
  double ctrl_dt = 0.005;         // 200 Hz 默认

  // LADRC 参数
  std::string control_mode = "pd";  // "pd" 或 "ladrc"
  Vec3d ladrc_b0{1.0, 1.0, 1.0};   // 各轴输入增益
  Vec3d ladrc_wc{2.0, 2.0, 3.0};   // 控制器带宽
  Vec3d ladrc_wo{10.0, 10.0, 15.0}; // 观测器带宽
};

// 将马达 ω² 转为 RPM
inline std::array<double, 4> motorSqToRpm(const MotorSq& u) {
  std::array<double, 4> rpm;
  for (int i = 0; i < 4; ++i) {
    double w2 = u(i);
    if (w2 < 0) w2 = 0;
    rpm[i] = std::sqrt(w2) * 60.0 / (2.0 * M_PI);
  }
  return rpm;
}

class DroneController {
 public:
  explicit DroneController(const Params& p) : params_(p) {
    if (p.mass <= 0) throw std::invalid_argument("Controller: mass <= 0");
    Binv_ = drone_common::buildMixerMatrixInverse(p.arm_length, p.k_F, p.k_M);
    ladrc_.configure(p.ladrc_b0, p.ladrc_wc, p.ladrc_wo, p.ctrl_dt);
  }

  const Params& params() const { return params_; }

  // 运行时热切换 LADRC 参数（供自动调参使用）
  void setLADRCParams(const Vec3d& b0, const Vec3d& wc, const Vec3d& wo) {
    params_.ladrc_b0 = b0;
    params_.ladrc_wc = wc;
    params_.ladrc_wo = wo;
    ladrc_.configure(b0, wc, wo, params_.ctrl_dt);
  }

  void setControlMode(const std::string& mode) { params_.control_mode = mode; }

  // 每控制周期调一次：odom 状态 + 目标位姿 → 4 RPM
  // odom_p = [x,y,z] 世界系
  // odom_v = [vx,vy,vz]
  // odom_q = q_WB
  // odom_w = ω_B
  // goal_p = 目标位置
  // goal_yaw = 目标偏航 rad
  std::array<double, 4> step(
      const Vec3d& odom_p, const Vec3d& odom_v,
      const Quatd& odom_q, const Vec3d& odom_w,
      const Vec3d& goal_p, double goal_yaw) {

    // 1. 位置误差 → 期望加速度
    Vec3d ep = goal_p - odom_p;
    Vec3d ev = -odom_v;  // v_des = 0 (hover / waypoint-hold)
    Vec3d a_des;

    if (params_.control_mode == "ladrc") {
      // LADRC 三轴独立计算（观测器已在内部更新）
      a_des = ladrc_.step(odom_p, goal_p);
    } else {
      // PD + 可选积分
      a_des = params_.Kp_pos.cwiseProduct(ep) + params_.Kd_pos.cwiseProduct(ev);
      ep_integral_ += ep * params_.ctrl_dt;
      ep_integral_ = ep_integral_.cwiseMin(params_.Ki_max).cwiseMax(-params_.Ki_max);
      if (params_.Ki_pos.norm() > 1e-9)
        a_des += params_.Ki_pos.cwiseProduct(ep_integral_);
    }

    // 加速度限幅 & 远目标 failsafe（两种模式共用）
    if (ep.norm() > params_.d_far) {
      a_des = ep.normalized() * params_.a_max_vec;
      ep_integral_.setZero();
    }
    a_des.x() = std::clamp(a_des.x(), -params_.a_xy_max, params_.a_xy_max);
    a_des.y() = std::clamp(a_des.y(), -params_.a_xy_max, params_.a_xy_max);
    a_des.z() = std::clamp(a_des.z(), params_.a_z_min,  params_.a_z_max);

    // 2. 期望推力矢量
    Vec3d az_w(0, 0, drone_common::kGravity);
    Vec3d T_W = params_.mass * (a_des + az_w);         // 世界系
    double F = T_W.norm();
    if (F < 1e-6) { T_W = az_w; F = params_.mass * drone_common::kGravity; }
    F = std::clamp(F, params_.F_min, params_.F_max);
    Vec3d z_B_des = T_W.normalized();

    // 3. 期望旋转矩阵 R_des（由 z_B_des + yaw 构造，避免 Euler 奇异）
    double sy = std::sin(goal_yaw), cy = std::cos(goal_yaw);
    Vec3d x_c(cy, sy, 0);                              // 世界水平面投影
    Vec3d y_B_des = z_B_des.cross(x_c);
    double ny = y_B_des.norm();
    if (ny < 1e-9) {
      // gimbal flip 附近退化：x_c 选与世界 x 正交替代
      x_c = Vec3d(1, 0, 0);
      y_B_des = z_B_des.cross(x_c);
      if (y_B_des.norm() < 1e-9) y_B_des = Vec3d(0, 1, 0);
      y_B_des.normalize();
    } else {
      y_B_des /= ny;
    }
    Vec3d x_B_des = y_B_des.cross(z_B_des).normalized();
    Mat3d R_des;
    R_des.col(0) = x_B_des;
    R_des.col(1) = y_B_des;
    R_des.col(2) = z_B_des;

    // 4. 几何姿态误差（Lee et al.）
    Mat3d R = odom_q.toRotationMatrix();
    Mat3d R_err = R_des.transpose() * R - R.transpose() * R_des;
    Vec3d e_R = 0.5 * drone_common::vee(R_err);           // 姿态误差
    Vec3d e_w = odom_w;                                   // ω_des = 0
    // 期望力矩：−Kp_att ∘ e_R − Kd_rate ∘ e_w
    Vec3d tau = -params_.Kp_att.cwiseProduct(e_R) - params_.Kd_rate.cwiseProduct(e_w);
    tau.x() = std::clamp(tau.x(), -params_.tau_max, params_.tau_max);
    tau.y() = std::clamp(tau.y(), -params_.tau_max, params_.tau_max);
    tau.z() = std::clamp(tau.z(), -params_.tau_max, params_.tau_max);

    // 5. mixer 逆
    Wrench w; w << F, tau.x(), tau.y(), tau.z();
    MotorSq u = Binv_ * w;
    // 非负 clamp（电机不反转）
    u = u.cwiseMax(0.0);
    // ω² clamp
    double w2_max = params_.omega_max * params_.omega_max;
    for (int i = 0; i < 4; ++i) u(i) = std::min(u(i), w2_max);

    return motorSqToRpm(u);
  }

  void resetIntegral() { ep_integral_.setZero(); }

 private:
  Params params_;
  Eigen::Matrix4d Binv_;
  Vec3d ep_integral_{0, 0, 0};
  LADRC3D ladrc_;
};

}  // namespace drone_controller
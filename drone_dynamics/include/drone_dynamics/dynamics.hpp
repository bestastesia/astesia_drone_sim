#pragma once
//
// dynamics.hpp — 四旋翼动力学模型（纯 C++，不依赖 ROS，便于单测）
// ----------------------------------------------------------------------
// 状态：位置 p_W、速度 v_W（世界系）、姿态四元数 q_WB（机体系→世界）、
//       机体系角速度 ω_B，外加 4 个电机转速 ω_i。
// 输入：4 个期望转速 ω_cmd_i（由 /drone/motor_rpm_cmd 的 RPM 转换）。
// 积分：1 ms 一步，半隐式 Euler + 四元数 normalize/tick。
//

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <array>
#include <cassert>
#include <cmath>
#include <random>

#include "drone_common/drone_common.hpp"

namespace drone_dynamics {

using drone_common::Vec3d;
using drone_common::Quatd;
using drone_common::Mat3d;

struct Params {
  double mass = 1.0;          // kg
  Vec3d inertia{0.02, 0.02, 0.04};  // [Ix,Iy,Iz] kg·m²
  double k_F = 1.0;           // N·s²/rad²
  double k_M = 0.05;          // N·m·s²/rad²
  double arm_length = 0.2;    // m
  double motor_tau = 0.02;   // 电机一阶时间常数 s
  double omega_min = 0.0;    // rad/s（0 = 允许停转）
  double omega_max = 1e3;    // rad/s
  double sim_dt = 0.001;     // s
  bool add_linear_drag = false;
  double drag_coeff = 0.1;   // 线性阻力 -c·v（若启用）
  // 风扰
  bool wind_enabled = false;
  Vec3d wind_force{0, 0, 0};  // 恒定风力 (世界系 N)
  double wind_gust_amplitude = 0.0;  // 阵风幅值 (N)
  double wind_gust_period = 2.0;     // 阵风周期 (s)
  double wind_gust_phase = 0.0;      // 阵风随机相位 (rad)
  Vec3d init_pos{0, 0, 0};
  double init_yaw = 0.0;
};

struct State {
  Vec3d p = Vec3d::Zero();        // 世界系位置
  Vec3d v = Vec3d::Zero();        // 世界系速度
  Quatd q = Quatd::Identity();    // 姿态 q_WB
  Vec3d w_body = Vec3d::Zero();   // 机体系角速度
  std::array<double, 4> motor_omega{0, 0, 0, 0};  // rad/s
  Vec3d a_world_last = Vec3d::Zero();  // 最近一步机体系→世界系加速度（供 IMU 特征力）
  Vec3d wind_total = Vec3d::Zero();     // 当前步风扰合力（供 ground station 展示）
};

// 从 RPM 命令转 ω_cmd (rad/s)
inline std::array<double, 4> rpmToOmegaCmd(const std::array<double, 4>& rpm_cmd) {
  std::array<double, 4> w;
  for (size_t i = 0; i < 4; ++i) w[i] = rpm_cmd[i] * 2.0 * M_PI / 60.0;
  return w;
}

class DroneDynamics {
 public:
  explicit DroneDynamics(const Params& p) : params_(p) {
    // 常驻参数校验（不依赖 NDEBUG/assert），守 Release 构建
    if (params_.mass <= 0 || params_.arm_length <= 0 ||
        params_.k_F <= 0 || params_.k_M <= 0 || params_.motor_tau <= 0) {
      throw std::invalid_argument("DroneDynamics: 非正物理参数");
    }
    B_ = drone_common::buildMixerMatrixB(params_.arm_length, params_.k_F, params_.k_M);
    I_body_ = params_.inertia.asDiagonal();
    // 重置到初始状态
    reset();
  }

  void reset() {
    state_ = State{};
    // 由 init_yaw 构初始姿态
    state_.q = Quatd(Eigen::AngleAxisd(params_.init_yaw, Vec3d::UnitZ()));
    state_.p = params_.init_pos;
    wind_time_ = 0.0;
  }

  const State& state() const { return state_; }
  const Params& params() const { return params_; }
  Vec3d totalWindForce() const { return wind_force_total_; }

  // 风扰 API
  void setWindEnabled(bool en) { params_.wind_enabled = en; }
  bool windEnabled() const { return params_.wind_enabled; }
  void setWindForce(const Vec3d& f) { params_.wind_force = f; }
  Vec3d windForce() const { return params_.wind_force; }
  void setWindGust(double amp, double period) {
    params_.wind_gust_amplitude = amp; params_.wind_gust_period = period;
  }

  // 用一组 RPM 命令推进一个 sim_dt。内部串 N 步子步以保持 ω 命令稳定（简单起见 1 步/dt）。
  void stepFromRpmCmd(const std::array<double, 4>& motor_rpm_cmd, double dt) {
    auto w_cmd = rpmToOmegaCmd(motor_rpm_cmd);
    stepFromOmegaCmd(w_cmd, dt);
  }

  // 用 ω 命令（rad/s）推进 dt（默认一次 sim_dt；外部可分多步）。
  void stepFromOmegaCmd(const std::array<double, 4>& omega_cmd, double dt) {
    // 电机一阶：ω̇_i = (ω_cmd_i − ω_i) / motor_tau
    // 半隐式 Euler：
    for (size_t i = 0; i < 4; ++i) {
      double wd = (omega_cmd[i] - state_.motor_omega[i]) / params_.motor_tau;
      state_.motor_omega[i] += wd * dt;
      if (state_.motor_omega[i] < params_.omega_min) state_.motor_omega[i] = params_.omega_min;
      if (state_.motor_omega[i] > params_.omega_max) state_.motor_omega[i] = params_.omega_max;
    }

    // 推力/力矩：u = [ω1²,ω2²,ω3²,ω4²]ᵀ，wrench = B u
    drone_common::MotorSq u;
    u << state_.motor_omega[0] * state_.motor_omega[0],
        state_.motor_omega[1] * state_.motor_omega[1],
        state_.motor_omega[2] * state_.motor_omega[2],
        state_.motor_omega[3] * state_.motor_omega[3];
    drone_common::Wrench wrench = B_ * u;
    double thrust = wrench(0);          // 总推力，沿机体 +z
    Vec3d torque_body(wrench(1), wrench(2), wrench(3));

    // 姿态矩阵 R_WB（把机体旋到世界）：用 q_WB
    Mat3d R_WB = state_.q.toRotationMatrix();
    // 平动：a_W = R_WB * (thrust/m) ẑ_B − g ẑ_W + wind (+ 可选阻力)
    Vec3d thrust_world = R_WB * Vec3d(0, 0, thrust);  // 机体 +z 旋到世界
    Vec3d gravity(0, 0, -drone_common::kGravity);
    Vec3d a_world = thrust_world / params_.mass + gravity;
    if (params_.add_linear_drag) a_world += -params_.drag_coeff * state_.v;

    // 风扰：恒定力 + 阵风（正弦 + 随机湍流）
    Vec3d wind_force_total = Vec3d::Zero();
    if (params_.wind_enabled) {
      wind_force_total = params_.wind_force;  // 恒定分量
      // 阵风：正弦调制 + Ornstein-Uhlenbeck 随机过程
      wind_time_ += dt;
      if (params_.wind_gust_amplitude > 1e-6) {
        double gust_det = params_.wind_gust_amplitude *
            std::sin(2.0 * M_PI * wind_time_ / params_.wind_gust_period
                     + params_.wind_gust_phase);
        // 随机湍流: dx = -θ·x·dt + σ·dW  (OU process → filtered white noise)
        // 用简单的 Euler-Maruyama 积分
        double theta = 1.0;   // 均值回复率 (≈ 1s 时间常数)
        double sigma = params_.wind_gust_amplitude * 0.5;  // 湍流强度
        // 生成随机噪声 (Box-Muller)
        double u1 = (wind_rng_() + 1.0) / (wind_rng_.max() + 2.0);
        double u2 = (wind_rng_() + 1.0) / (wind_rng_.max() + 2.0);
        if (u1 < 1e-10) u1 = 1e-10;
        double dw = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * M_PI * u2);
        wind_turbulence_ += -theta * wind_turbulence_ * dt + sigma * dw * std::sqrt(dt);
        // 总阵风 = 确定性 + 随机
        double gust_total = gust_det + wind_turbulence_;
        // 阵风沿恒定风方向施加
        double f_norm = params_.wind_force.norm();
        if (f_norm > 1e-6) {
          wind_force_total += params_.wind_force.normalized() * gust_total;
        } else {
          wind_force_total.x() += gust_total;
        }
      }
      a_world += wind_force_total / params_.mass;
    }
    state_.a_world_last = a_world;
    wind_force_total_ = wind_force_total;
    state_.wind_total = wind_force_total;

    // 转动：ω̇_B = I⁻¹ (τ − ω × (I ω))
    Vec3d Iw = I_body_ * state_.w_body;
    Vec3d wdot = I_body_.inverse() * (torque_body - state_.w_body.cross(Iw));

    // 半隐式 Euler 积分
    state_.v += a_world * dt;
    state_.p += state_.v * dt;
    state_.w_body += wdot * dt;
    state_.q = drone_common::integrateQuaternion(state_.q, state_.w_body, dt);
  }

  double wind_time_ = 0.0;
  Vec3d wind_force_total_ = Vec3d::Zero();
  double wind_turbulence_ = 0.0;
  std::mt19937_64 wind_rng_{42};  // 风扰随机数生成器

 private:
  Params params_;
  State state_;
  Eigen::Matrix4d B_;
  Mat3d I_body_;
};

}  // namespace drone_dynamics
#pragma once
//
// mixer.hpp — 控制分配矩阵 B 与其逆 B⁻¹
// ----------------------------------------------------------------------
// 状态输入 u = [ω1², ω2², ω3², ω4²]ᵀ (rad²/s²)，电机顺序 [FL, FR, BL, BR]。
// 输出 wrench w = [F, τx, τy, τz]ᵀ：
//   F   总推力 (沿机体 +z)，单位 N
//   τx  绕机体 x 的力矩 (roll)，  N·m
//   τy  绕机体 y 的力矩 (pitch)， N·m
//   τz  绕机体 z 的力矩 (yaw)，   N·m
// w = B u。控制器用 u = B⁻¹ w 反解出 4 个 ω²。
//
// 单电机：
//   推力 F_i = k_F * ω_i² ，方向沿机体 +z
//   反扭矩 M_z,i = k_M * sgn_i * ω_i² ，sgn 见 frames.hpp
//
// 力矩来自两路：
//   (1) 推力对臂的叉乘贡献 roll/pitch：
//       τ_thrust = Σ r_i × (F_i ẑ) = Σ (r_yi F_i, -r_xi F_i, 0)
//       因 r_i = l/√2 * (dirX_i, dirY_i, 0)，得
//       τx = Σ r_yi F_i = (l/√2) k_F Σ dirY_i ω_i²
//       τy = Σ -r_xi F_i = -(l/√2) k_F Σ dirX_i ω_i²
//       （注意 -r_xi = -(l/√2) dirX_i，τy 行符号含负）
//   (2) 反扭矩贡献 yaw：τz = Σ k_M sgn_i ω_i²
//
// 所以 B (行 = [F, τx, τy, τz]，列 = [ω1²..ω4²])：
//   F   行: k_F          * [1   1   1   1 ]
//   τx  行: (l/√2) k_F  * [dirY]   = [+1, -1, +1, -1] * (l/√2) k_F
//   τy  行: -(l/√2) k_F * [dirX]   = -[+1, +1, -1, -1] (l/√2) k_F
//   τz  行: k_M         * [sgn]    = [+1, -1, -1, +1] * k_M
//
// frames.hpp: dirY = [+1,-1,+1,-1], dirX = [+1,+1,-1,-1], sgn = [+1,-1,-1,+1]
// 验证 τx 列 = dirY，τz 列 = sgn，二者这里恰好相同（PX4 X 配置下成立）。
//

#include <Eigen/Dense>
#include <array>
#include <cassert>

namespace drone_common {

using Wrench = Eigen::Vector4d;        // [F, τx, τy, τz]
using MotorSq = Eigen::Vector4d;       // [ω1², ω2², ω3², ω4²]

// 由物理参数构建 4×4 控制分配矩阵 B。
// arm_length 单位 m，k_F 单位 N·s²/rad²，k_M 单位 N·m·s²/rad²。
inline Eigen::Matrix4d buildMixerMatrixB(double arm_length, double k_F, double k_M) {
  // 参数 sanity：负或零会致 B 奇异/符号翻转 → 炸机。Debug 构建会触发，Release 仍跑（由调用方兜底）。
  assert(k_F > 0.0);
  assert(k_M > 0.0);
  assert(arm_length > 0.0);
  const double l = arm_length;
  const double a = l / std::sqrt(2.0) * k_F;  // roll/pitch 臂系数
  Eigen::Matrix4d B;
  // 行 0: F
  B.row(0) << k_F, k_F, k_F, k_F;
  // 行 1: τx = (l/√2) k_F * dirY = a * [+1, -1, +1, -1]
  B.row(1) << a, -a, a, -a;
  // 行 2: τy = -(l/√2) k_F * dirX = -a * [+1, +1, -1, -1]
  B.row(2) << -a, -a, a, a;
  // 行 3: τz = k_M * sgn = k_M * [+1, -1, -1, +1]
  B.row(3) << k_M, -k_M, -k_M, k_M;
  return B;
}

// 返回 B⁻¹。B 是常数 4×4 且非奇异（X 配置），这里直接求逆。
// 调用方应在启动时算一次缓存。
inline Eigen::Matrix4d buildMixerMatrixInverse(double arm_length, double k_F, double k_M) {
  return buildMixerMatrixB(arm_length, k_F, k_M).inverse();
}

// 给定 wrench [F, τx, τy, τz]，反解 4 个 ω²。
// ω² 物理上必须非负（电机不能反转）；B⁻¹ 在低速大 yaw 等工况可能解出负值，
// 这里 cwiseMax(0) 兜底——等价于"该电机停转"，代价是 yaw authority 下降（README 注明）。
inline MotorSq wrenchToMotorSq(const Eigen::Matrix4d& B_inv, const Wrench& w) {
  MotorSq u = B_inv * w;
  u = u.cwiseMax(0.0);
  return u;
}

// 给定 4 个 ω²，得到 wrench。
inline Wrench motorSqToWrench(const Eigen::Matrix4d& B, const MotorSq& u) {
  return B * u;
}

}  // namespace drone_common
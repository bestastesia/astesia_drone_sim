#pragma once
//
// math.hpp — Eigen 上的姿态/运动学小工具
// ----------------------------------------------------------------------
// 四元数惯例：Eigen::Quaterniond，内部存储 [x,y,z,w]，构造 (w,x,y,z)。
// 旋转 R_WB 作用方式：v_W = R_WB * v_B（把机体向量转到世界）。
// 四元数 q_WB 同义：v_W = q_WB * v_B。
//

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace drone_common {

using Vec3d = Eigen::Vector3d;
using Quatd = Eigen::Quaterniond;
using Mat3d = Eigen::Matrix3d;

// skew(w)∗v = w × v
inline Mat3d skew(const Vec3d& w) {
  Mat3d S;
  S <<     0, -w.z(),  w.y(),
        w.z(),     0, -w.x(),
       -w.y(),  w.x(),     0;
  return S;
}

// 四元数一次积分：
//   q̇ = 0.5 * q ⊗ Ω(ω)  ，Ω = [0, ωx, ωy, ωz]（纯虚四元数）
//   一阶显式：q_new = normalize(q + 0.5*dt * (q ⊗ Ω))
// ω 为机体系角速度。1 kHz 下显式 + 每步归一化足够（幅度精度）。
inline Quatd integrateQuaternion(const Quatd& q, const Vec3d& omega_body, double dt) {
  // Ω 作为 (w=0, xyz=ω) 的四元数
  Quatd omega_q(0.0, omega_body.x(), omega_body.y(), omega_body.z());
  Quatd qdot = q * omega_q;              // 注意：q ⊗ Ω
  Eigen::Vector4d dq(qdot.w(), qdot.x(), qdot.y(), qdot.z());
  Eigen::Vector4d qv(q.w(), q.x(), q.y(), q.z());
  qv += 0.5 * dt * dq;
  Quatd q_new(qv(0), qv(1), qv(2), qv(3));
  q_new.normalize();
  return q_new;
}

// 由机体系角速度转向量构建期望 R_des 角度时的 setpoint 用（见 controller）。
// 从 (z_B_des 单位向量, yaw ψ) 构造 R_des，避免 Euler 奇异。
// 返回列向量构成 R_des = [x_B_des | y_B_des | z_B_des] 的旋转矩阵（行优先存储）。
inline Mat3d rotationFromBodyZAndYaw(const Vec3d& z_body_des, double yaw) {
  Vec3d zbd = z_body_des.normalized();
  // 世界系投影：选 x_C = [cos ψ, sin ψ, 0]（水平面内，指向偏航方向）
  Vec3d x_c(std::cos(yaw), std::sin(yaw), 0.0);
  // y_B_des = z_B_des × x_C，再归一化
  Vec3d y_bd = zbd.cross(x_c).normalized();
  // x_B_des = y_B_des × z_B_des
  Vec3d x_bd = y_bd.cross(zbd).normalized();
  Mat3d R;
  R.col(0) = x_bd;
  R.col(1) = y_bd;
  R.col(2) = zbd;
  return R;
}

// 从 R 取四元数（行优先 Eigen 内部布局）
inline Quatd quatFromRotation(const Mat3d& R) {
  return Quatd(R).normalized();
}

// 旋转矩阵的 vee 映射：vee(skew-like) 用于几何姿态误差 e_R = 0.5*vee(R_desᵀR − RᵀR_des)
// 上式是反对称矩阵，取其 (2,1),(0,2),(1,0) 即可。
inline Vec3d vee(const Mat3d& M) {
  // M 假定反对称：M = -Mᵀ
  return Vec3d(M(2, 1), M(0, 2), M(1, 0));
}

}  // namespace drone_common
#pragma once
//
// ladrc.hpp — 二阶线性自抗扰控制器 (LADRC)，每轴独立
// ----------------------------------------------------------------------
// 模型: ẍ = f + b₀·u
// LESO: 观测 z₁=pos, z₂=vel, z₃=总扰动 f
// LSEF: u₀ = ωc²·(r-z₁) + 2ωc·(ṙ-z₂)
// 控制: u = (u₀ - z₃) / b₀
//
// 带宽参数化: kp=ωc², kd=2ωc, β₁=3ωo, β₂=3ωo², β₃=ωo³
//
// 参考: Gao, "Scaling and Bandwidth-Parameterization Based Controller Tuning"
//       https://doi.org/10.1109/ACC.2003.1242516

#include <cmath>
#include <algorithm>

namespace drone_controller {

class LADRC1D {
 public:
  LADRC1D() = default;

  // b0: 输入增益（≈ 系统惯性 1/mass 量级）
  // wc: 控制器带宽 rad/s（越大越快，受噪声/执行器速率限制）
  // wo: 观测器带宽 rad/s（典型值 3~5× wc）
  // dt: 控制周期 s
  void configure(double b0, double wc, double wo, double dt) {
    b0_ = b0;
    wc_ = wc;
    wo_ = wo;
    dt_ = dt;
    // LSEF gains
    kp_ = wc * wc;          // ωc²
    kd_ = 2.0 * wc;         // 2ωc
    // LESO gains (bandwidth parameterization)
    beta1_ = 3.0 * wo;
    beta2_ = 3.0 * wo * wo;
    beta3_ = wo * wo * wo;
    reset();
  }

  void reset() {
    z1_ = z2_ = z3_ = 0.0;
  }

  // 设置初始状态（避免观测器从 0 收敛的瞬态）
  void setInitialState(double y0) {
    z1_ = y0;
    z2_ = 0.0;
    z3_ = 0.0;
  }

  // 每控制周期调一次
  // y: 当前位置测量
  // r: 参考位置
  // r_dot: 参考速度（通常 0）
  // 返回: 控制量 u（加速度命令）
  double step(double y, double r, double r_dot = 0.0) {
    // ---- LESO (Euler forward) ----
    double e = z1_ - y;
    z1_ += dt_ * (z2_ - beta1_ * e);
    z2_ += dt_ * (z3_ - beta2_ * e + b0_ * last_u_);
    z3_ += dt_ * (-beta3_ * e);

    // ---- LSEF ----
    double u0 = kp_ * (r - z1_) + kd_ * (r_dot - z2_);

    // ---- 扰动补偿 ----
    double u = (u0 - z3_) / b0_;
    last_u_ = u;
    return u;
  }

  // 访问观测状态（调试用）
  double z1() const { return z1_; }
  double z2() const { return z2_; }
  double z3() const { return z3_; }

 private:
  double b0_ = 1.0, wc_ = 2.0, wo_ = 10.0, dt_ = 0.005;
  double kp_ = 4.0, kd_ = 4.0;
  double beta1_ = 30.0, beta2_ = 300.0, beta3_ = 1000.0;

  double z1_ = 0, z2_ = 0, z3_ = 0;
  double last_u_ = 0;
};

// 三轴 LADRC 包装
struct LADRC3D {
  LADRC1D x, y, z;

  void configure(const Eigen::Vector3d& b0, const Eigen::Vector3d& wc,
                 const Eigen::Vector3d& wo, double dt) {
    x.configure(b0.x(), wc.x(), wo.x(), dt);
    y.configure(b0.y(), wc.y(), wo.y(), dt);
    z.configure(b0.z(), wc.z(), wo.z(), dt);
  }

  void reset() { x.reset(); y.reset(); z.reset(); }

  void setInitialState(const Eigen::Vector3d& p0) {
    x.setInitialState(p0.x());
    y.setInitialState(p0.y());
    z.setInitialState(p0.z());
  }

  // 返回加速度命令 [ax, ay, az]
  Eigen::Vector3d step(const Eigen::Vector3d& pos, const Eigen::Vector3d& ref,
                       const Eigen::Vector3d& ref_dot = Eigen::Vector3d::Zero()) {
    return Eigen::Vector3d(
        x.step(pos.x(), ref.x(), ref_dot.x()),
        y.step(pos.y(), ref.y(), ref_dot.y()),
        z.step(pos.z(), ref.z(), ref_dot.z()));
  }
};

}  // namespace drone_controller
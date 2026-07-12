// drone_common 单元测试：mixer、math 的回归断言。
// 由 ament_cmake_gtest 在 `colcon test` 下跑；也可脱离 ROS 单独 g++ 编译跑。
#include <gtest/gtest.h>
#include <cmath>
#include "drone_common/mixer.hpp"
#include "drone_common/math.hpp"

using drone_common::Wrench;
using drone_common::MotorSq;

static double tol = 1e-9;
static bool approx(double a, double b, double e = 1e-9) {
  return std::abs(a - b) <= e * (1.0 + std::abs(b));
}

TEST(Mixer, BInverseIsIdentity) {
  const double l = 0.2, kF = 1.0, kM = 0.05;
  Eigen::Matrix4d B = drone_common::buildMixerMatrixB(l, kF, kM);
  Eigen::Matrix4d Binv = drone_common::buildMixerMatrixInverse(l, kF, kM);
  Eigen::Matrix4d I = B * Binv;
  for (int r = 0; r < 4; ++r)
    for (int c = 0; c < 4; ++c)
      EXPECT_NEAR(I(r, c), (r == c) ? 1.0 : 0.0, 1e-9);
}

TEST(Mixer, HoverAllEqual) {
  const double l = 0.2, kF = 1.0, kM = 0.05;
  auto Binv = drone_common::buildMixerMatrixInverse(l, kF, kM);
  Wrench w; w << 4 * kF, 0, 0, 0;
  MotorSq u = drone_common::wrenchToMotorSq(Binv, w);
  for (int i = 1; i < 4; ++i) EXPECT_NEAR(u(i), u(0), tol);
}

TEST(Mixer, RollTorqueSign) {
  // +roll → 前左/后左(r_y>0)快于前右/后右
  const double l = 0.2, kF = 1.0, kM = 0.05;
  auto Binv = drone_common::buildMixerMatrixInverse(l, kF, kM);
  Wrench w; w << 0, 1.0, 0, 0;
  MotorSq u = drone_common::wrenchToMotorSq(Binv, w);
  EXPECT_GT(u(0), u(1));  // FL > FR
  EXPECT_GT(u(2), u(3));  // BL > BR
}

TEST(Mixer, YawTorqueSign) {
  // +yaw → CCW 电机 (m1,m4) 快
  const double l = 0.2, kF = 1.0, kM = 0.05;
  auto Binv = drone_common::buildMixerMatrixInverse(l, kF, kM);
  Wrench w; w << 0, 0, 0, 1.0;
  MotorSq u = drone_common::wrenchToMotorSq(Binv, w);
  EXPECT_GT(u(0), u(1));  // FL > FR
  EXPECT_GT(u(3), u(2));  // BR > BL
}

TEST(Mixer, NegativeOmegaSquaredClamped) {
  // 纯力矩 wrench 会从 B⁻¹ 解出负 ω²；wrenchToMotorSq 必须 clamp 到 ≥0，
  // 不能把负值传给 sqrt（否则 NaN 炸机）。
  const double l = 0.2, kF = 1.0, kM = 0.05;
  auto Binv = drone_common::buildMixerMatrixInverse(l, kF, kM);
  Wrench w; w << 0, 1.0, 0, 0;  // 纯 +roll
  MotorSq u = drone_common::wrenchToMotorSq(Binv, w);
  for (int i = 0; i < 4; ++i) EXPECT_GE(u(i), 0.0);
  // 还要确认真有 clamping 发生（至少一个被压到 0），否则这条测试是哑的
  int zeros = 0;
  for (int i = 0; i < 4; ++i) if (u(i) == 0.0) ++zeros;
  EXPECT_GT(zeros, 0) << "纯力矩应使某些 ω² 为负再被 clamp 到 0";
}

TEST(Math, QuaternionIntegrateZAxis) {
  drone_common::Quatd q = drone_common::Quatd::Identity();
  drone_common::Vec3d omega(0, 0, 0.5);
  double dt = 0.001;
  for (int i = 0; i < 1000; ++i) q = drone_common::integrateQuaternion(q, omega, dt);
  drone_common::Vec3d xw = q * drone_common::Vec3d(1, 0, 0);
  EXPECT_NEAR(xw.x(), std::cos(0.5), 1e-3);
  EXPECT_NEAR(xw.y(), std::sin(0.5), 1e-3);
  EXPECT_NEAR(xw.z(), 0.0, 1e-3);
  EXPECT_NEAR(q.norm(), 1.0, 1e-6);
}

TEST(Math, RotationFromZAndYaw) {
  auto R0 = drone_common::rotationFromBodyZAndYaw(drone_common::Vec3d(0, 0, 1), 0.0);
  for (int r = 0; r < 3; ++r)
    for (int c = 0; c < 3; ++c)
      EXPECT_NEAR(R0(r, c), (r == c) ? 1.0 : 0.0, 1e-9);
  auto Ry = drone_common::rotationFromBodyZAndYaw(drone_common::Vec3d(0, 0, 1), M_PI / 2);
  drone_common::Vec3d xB = Ry.col(0);
  EXPECT_NEAR(xB.x(), 0.0, 1e-9);
  EXPECT_NEAR(xB.y(), 1.0, 1e-9);
  EXPECT_NEAR(xB.z(), 0.0, 1e-9);
}

TEST(Math, VeeSkewInverse) {
  drone_common::Vec3d v(1.1, -2.2, 3.3);
  EXPECT_LT((drone_common::vee(drone_common::skew(v)) - v).norm(), 1e-9);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
#pragma once
//
// frames.hpp — 坐标系与电机布局的单一事实来源
// ----------------------------------------------------------------------
// 世界系 map (REP-103 ENU)：+x 前/东, +y 左/北, +z 上, 重力 g 沿 -z。
// 机体系 base_link：+x 前, +y 左, +z 上，右手系 (x × y = +z)。
//
// X 型四旋翼，4 电机在机体 xy 对角，距中心 arm_length l：
//   俯视 (沿 -z 看)：
//        前 (+x)
//        ^
//   m1 FL ──── m2 FR      左上 m1(前左) — 右上 m2(前右)
//      \   /              旋向：FL=CCW(+), FR=CW(-)
//       \ /
//        X
//       / \
//      /   \              左下 m3(后左) — 右下 m4(后右)
//   m3 BL ──── m4 BR      旋向：BL=CW(-), BR=CCW(+)
//
// 对角同向（PX4 风格）：(m1,m4) 同 CCW(+)，(m2,m3) 同 CW(-)。
// 反扭矩（yaw）方向 = 旋向：CCW 电机对机体产生 +z 反扭矩，CW 产生 -z。
//
// 电机机体系位置 r_i (arm_length = l)：
//   m1 FL : (+l/√2, +l/√2, 0)   spin +
//   m2 FR : (+l/√2, -l/√2, 0)   spin -
//   m3 BL : (-l/√2, +l/√2, 0)   spin -
//   m4 BR : (-l/√2, -l/√2, 0)   spin +
//
// 所有节点必须从这里取电机索引与方向，禁止各自硬编码。
//

#include <cstddef>
#include <array>

namespace drone_common {

// 电机索引（与 /drone/motor_rpm_cmd 的 Float32MultiArray.data 顺序一致）
// data = [FL, FR, BL, BR]
enum class MotorIndex : std::size_t {
  FL = 0,  // front-left,  CCW (+)
  FR = 1,  // front-right, CW  (-)
  BL = 2,  // back-left,   CW  (-)
  BR = 3,  // back-right,  CCW (+)
};

// 反扭矩/反作用力矩方向符号：+1 = CCW，-1 = CW
// 索引顺序与 MotorIndex 一致 [FL, FR, BL, BR]
constexpr std::array<double, 4> kMotorYawSign = {+1.0, -1.0, -1.0, +1.0};

// 电机机体系 xy 笼坐标方向（不含 arm_length 缩放，乘以 l 后得真实坐标）
// 索引顺序与 MotorIndex 一致
constexpr std::array<double, 4> kMotorDirX = {+1.0, +1.0, -1.0, -1.0};  // +x 前
constexpr std::array<double, 4> kMotorDirY = {+1.0, -1.0, +1.0, -1.0};  // +y 左
// 注：FL=(+,+), FR=(+,-), BL=(-,+), BR=(-,-) —— 与上方俯视图一致。

// 物理常数 (SI)
constexpr double kGravity = 9.80665;  // m/s^2，沿世界 -z

// 四元数惯例: Eigen::Quaterniond (w,x,y,z) 内部存储为 [x,y,z,w]
// 这里统一用 Eigen，不自己存数组，避免顺序坑。

}  // namespace drone_common
#pragma once
//
// noise.hpp — 传感器噪声模型（纯 C++，不依赖 ROS）
// ----------------------------------------------------------------------
// IMU 模型：加速度计/陀螺仪偏置 + 高斯白噪声 + 偏置随机游走
// Odom 模型：位置/速度高斯加性噪声
//
// 用法：
//   ImuNoiseModel imu(0.01, 0.001, 1e-5, 0.01, 0.001, 1e-5, 12345);
//   Vec3d accel_noisy = imu.corruptAccel(true_accel, dt);
//   Vec3d gyro_noisy  = imu.corruptGyro(true_gyro, dt);
//

#include <Eigen/Dense>
#include <random>
#include <cmath>

namespace drone_common {

using Vec3d = Eigen::Vector3d;

class ImuNoiseModel {
 public:
  // accel_noise_density: accel 白噪声标准差 m/s²
  // accel_bias_init: 初始偏置 m/s²
  // accel_bias_rw: 偏置随机游走标准差 m/s²/√s
  // gyro_noise_density: gyro 白噪声 rad/s
  // gyro_bias_init: 初始偏置 rad/s
  // gyro_bias_rw: 偏置随机游走 rad/s/√s
  // seed: 随机种子
  ImuNoiseModel(double accel_noise_density, double accel_bias_init,
                double accel_bias_rw,
                double gyro_noise_density, double gyro_bias_init,
                double gyro_bias_rw, uint64_t seed = 12345)
    : accel_nd_(accel_noise_density), accel_bias_(accel_bias_init, accel_bias_init, accel_bias_init),
      accel_bias_rw_(accel_bias_rw),
      gyro_nd_(gyro_noise_density), gyro_bias_(gyro_bias_init, gyro_bias_init, gyro_bias_init),
      gyro_bias_rw_(gyro_bias_rw), rng_(seed) {}

  // 污染加速度计读数（输入真实比力，输出含噪值）
  Vec3d corruptAccel(const Vec3d& true_accel, double dt) {
    Vec3d noise;
    for (int i = 0; i < 3; ++i) noise[i] = accel_nd_ * gauss_(rng_);
    // bias random walk: b += σ_rw·√dt·N(0,1)
    for (int i = 0; i < 3; ++i)
      accel_bias_[i] += accel_bias_rw_ * std::sqrt(dt) * gauss_(rng_);
    return true_accel + accel_bias_ + noise;
  }

  Vec3d corruptGyro(const Vec3d& true_gyro, double dt) {
    Vec3d noise;
    for (int i = 0; i < 3; ++i) noise[i] = gyro_nd_ * gauss_(rng_);
    for (int i = 0; i < 3; ++i)
      gyro_bias_[i] += gyro_bias_rw_ * std::sqrt(dt) * gauss_(rng_);
    return true_gyro + gyro_bias_ + noise;
  }

  Vec3d accelBias() const { return accel_bias_; }
  Vec3d gyroBias() const { return gyro_bias_; }

 private:
  double accel_nd_, accel_bias_rw_, gyro_nd_, gyro_bias_rw_;
  Vec3d accel_bias_, gyro_bias_;
  std::mt19937_64 rng_;
  std::normal_distribution<double> gauss_{0.0, 1.0};
};

// Odom 加性高斯噪声：位置 σ_pos m, 速度 σ_vel m/s
inline Vec3d addPositionNoise(const Vec3d& true_pos, double sigma_pos,
                               std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, sigma_pos);
  return true_pos + Vec3d(gauss(rng), gauss(rng), gauss(rng));
}

inline Vec3d addVelocityNoise(const Vec3d& true_vel, double sigma_vel,
                               std::mt19937_64& rng) {
  std::normal_distribution<double> gauss(0.0, sigma_vel);
  return true_vel + Vec3d(gauss(rng), gauss(rng), gauss(rng));
}

}  // namespace drone_common

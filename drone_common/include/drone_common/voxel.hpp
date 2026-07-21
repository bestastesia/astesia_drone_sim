#pragma once
//
// voxel.hpp — 3D 体素栅格 + FOV 锥体局部感知模拟
// 纯 C++，不依赖 ROS，可直接包含用于 planner 或测试
// --------------------------------------------------------------------------
// 用法:
//   VoxelGrid grid(bounds_min, bounds_max, resolution);
//   grid.markObstacles(obstacles, inflation);
//   bool free = grid.isFree(center, half_extents);
//   Grid3D local = grid.queryFOV(origin, direction, fov_h, fov_v, range);
//

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>

namespace drone_common {

using Vec3d = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;

class VoxelGrid {
 public:
  // bounds: [min_x, min_y, min_z, max_x, max_y, max_z]
  VoxelGrid(const std::vector<double>& bounds, double resolution)
    : res_(resolution) {
    inv_res_ = 1.0 / res_;
    origin_ << bounds[0], bounds[1], bounds[2];
    max_corner_ << bounds[3], bounds[4], bounds[5];
    Vec3d size = max_corner_ - origin_;
    dims_ = Vec3i(
      static_cast<int>(std::ceil(size.x() * inv_res_)),
      static_cast<int>(std::ceil(size.y() * inv_res_)),
      static_cast<int>(std::ceil(size.z() * inv_res_))
    );
    cells_.resize(dims_.x() * dims_.y() * dims_.z(), 0);
  }

  // World → grid index
  Vec3i worldToGrid(const Vec3d& p) const {
    return Vec3i(
      static_cast<int>((p.x() - origin_.x()) * inv_res_),
      static_cast<int>((p.y() - origin_.y()) * inv_res_),
      static_cast<int>((p.z() - origin_.z()) * inv_res_)
    );
  }

  // Grid index → world center
  Vec3d gridToWorld(const Vec3i& g) const {
    return origin_ + Vec3d(g.x() + 0.5, g.y() + 0.5, g.z() + 0.5) * res_;
  }

  int& at(int x, int y, int z) {
    return cells_[index(x, y, z)];
  }

  int at(int x, int y, int z) const {
    return cells_[index(x, y, z)];
  }

  bool inBounds(int x, int y, int z) const {
    return x >= 0 && x < dims_.x() &&
           y >= 0 && y < dims_.y() &&
           z >= 0 && z < dims_.z();
  }

  bool inBounds(const Vec3i& g) const {
    return inBounds(g.x(), g.y(), g.z());
  }

  bool isOccupied(const Vec3i& g) const {
    return inBounds(g) && at(g.x(), g.y(), g.z()) != 0;
  }

  // 用 sphere 标记占用（给定中心、半径、膨胀）
  void markSphere(const Vec3d& center, double radius, double inflation) {
    double r_total = radius + inflation;
    int r_cells = static_cast<int>(std::ceil(r_total * inv_res_));
    Vec3i gc = worldToGrid(center);
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dz = -r_cells; dz <= r_cells; ++dz) {
          int gx = gc.x() + dx, gy = gc.y() + dy, gz = gc.z() + dz;
          if (!inBounds(gx, gy, gz)) continue;
          Vec3d wc = gridToWorld(Vec3i(gx, gy, gz));
          double dist = (wc - center).norm();
          if (dist <= r_total) {
            at(gx, gy, gz) = 1;
          }
        }
      }
    }
  }

  // 用 cylinder 标记（垂直柱体）
  void markCylinder(const Vec3d& center, double radius, double height,
                    double inflation) {
    double r_total = radius + inflation;
    int r_cells = static_cast<int>(std::ceil(r_total * inv_res_));
    double half_h = height * 0.5 + inflation;
    Vec3i gc = worldToGrid(center);
    int h_cells = static_cast<int>(std::ceil(half_h * inv_res_));
    for (int dx = -r_cells; dx <= r_cells; ++dx) {
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dz = -h_cells; dz <= h_cells; ++dz) {
          int gx = gc.x() + dx, gy = gc.y() + dy, gz = gc.z() + dz;
          if (!inBounds(gx, gy, gz)) continue;
          Vec3d wc = gridToWorld(Vec3i(gx, gy, gz));
          double hdist = std::abs(wc.z() - center.z());
          double rdist = (wc.head<2>() - center.head<2>()).norm();
          if (rdist <= r_total && hdist <= half_h) {
            at(gx, gy, gz) = 1;
          }
        }
      }
    }
  }

  // ====================================================================
  // FOV 锥体感知: 从 origin 沿 direction 方向, 返回可见的 3D grid 切片
  // ====================================================================
  // fov_h: 水平视场角 (rad)
  // fov_v: 垂直视场角 (rad)
  // range: 最大感知距离 (m)
  // direction: 机身前方 (机体系 +x)
  struct FOVResult {
    std::vector<char> visible;  // 3D flattened, same dims as grid
    int nx, ny, nz;
    double res;
    Vec3d origin;
  };

  FOVResult queryFOV(const Vec3d& observer, const Vec3d& direction,
                      double fov_h, double fov_v, double range) const {
    FOVResult result;
    result.nx = dims_.x(); result.ny = dims_.y(); result.nz = dims_.z();
    result.res = res_; result.origin = origin_;
    result.visible.resize(cells_.size(), 0);

    double half_h = fov_h * 0.5;
    double half_v = fov_v * 0.5;
    Vec3d dir_norm = direction.normalized();

    for (int x = 0; x < dims_.x(); ++x) {
      for (int y = 0; y < dims_.y(); ++y) {
        for (int z = 0; z < dims_.z(); ++z) {
          Vec3d wc = gridToWorld(Vec3i(x, y, z));
          Vec3d to_cell = wc - observer;
          double dist = to_cell.norm();
          if (dist > range) continue;

          // Check if within FOV cone
          Vec3d to_norm = to_cell.normalized();
          double cos_angle = to_norm.dot(dir_norm);
          // Project onto horizontal and vertical
          double h_angle = std::acos(std::max(-1.0, std::min(1.0, cos_angle)));
          if (h_angle > std::max(half_h, half_v)) continue;

          // Simple cone check: angle between direction and to_cell
          if (std::acos(std::max(-1.0, std::min(1.0, cos_angle))) >
              std::max(half_h, half_v)) continue;

          result.visible[index(x, y, z)] = 1;
        }
      }
    }
    return result;
  }

  // Getters
  Vec3i dims() const { return dims_; }
  double res() const { return res_; }
  const std::vector<int>& cells() const { return cells_; }
  size_t occupiedCount() const {
    size_t n = 0;
    for (int v : cells_) if (v) ++n;
    return n;
  }

 private:
  size_t index(int x, int y, int z) const {
    return static_cast<size_t>(x + dims_.x() * (y + dims_.y() * z));
  }

  Vec3d origin_, max_corner_;
  double res_, inv_res_;
  Vec3i dims_;
  std::vector<int> cells_;
};

}  // namespace drone_common

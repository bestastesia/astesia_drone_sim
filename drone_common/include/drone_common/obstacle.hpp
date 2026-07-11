#pragma once
//
// obstacle.hpp — 障碍物表示与几何助手（map 与 planner 共用）
// ----------------------------------------------------------------------
// 只支持轴对齐几何（无朝向），简化碰撞检测与栅格膨胀。
//

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <string>
#include <vector>

namespace drone_common {

using Vec3d = Eigen::Vector3d;

enum class Shape { Sphere, Cylinder, Cube };

struct Obstacle {
  Shape shape;
  Vec3d center{0, 0, 0};
  double radius = 0.0;        // sphere / cylinder (xy 半径)
  double height = 0.0;        // cylinder: 沿 z 的高度（含中心 z 上下各 height/2）
  Vec3d half_extents{0, 0, 0}; // cube: 各轴半长
};

// 2D 膨胀半径：在 xy 平面上，把障碍物外形向外推 inflate。
// 球/柱 → 半径 r + inflate 的圆盘；立方 → 各方向 half_extent + inflate 的 AABB。
// 单位 m。
struct Inflate2D {
  double radius = 0.0;              // 若为圆盘占用
  bool is_disk = true;
  Eigen::Vector2d aabb_min{0, 0};    // 若为 AABB 占用
  Eigen::Vector2d aabb_max{0, 0};
};

inline Inflate2D footprint2D(const Obstacle& o, double inflate) {
  Inflate2D f;
  switch (o.shape) {
    case Shape::Sphere:
    case Shape::Cylinder: {
      f.is_disk = true;
      f.radius = o.radius + inflate;
      const double cx = o.center.x(), cy = o.center.y();
      f.aabb_min << cx - f.radius, cy - f.radius;
      f.aabb_max << cx + f.radius, cy + f.radius;
      break;
    }
    case Shape::Cube: {
      f.is_disk = false;
      f.aabb_min << o.center.x() - o.half_extents.x() - inflate,
                   o.center.y() - o.half_extents.y() - inflate;
      f.aabb_max << o.center.x() + o.half_extents.x() + inflate,
                   o.center.y() + o.half_extents.y() + inflate;
      // 用 AABB 外接圆半径近似，便于一些粗筛
      f.radius = (o.half_extents.head<2>().norm() + inflate);
      break;
    }
  }
  return f;
}

// 该障碍物在 z 高度处是否“有意义”：cylinder/cube 的 z 跨度是否包含 z_query。
// sphere 视为点障碍（z 任意都占用，因为球在所有高度都有截面）。
inline bool zOverlap(const Obstacle& o, double z_query, double /*inflate_z*/ = 0.0) {
  switch (o.shape) {
    case Shape::Sphere:
      // 球：只要 query 在 [c-r, c+r] 范围
      return std::abs(z_query - o.center.z()) <= o.radius;
    case Shape::Cylinder:
      return std::abs(z_query - o.center.z()) <= 0.5 * o.height;
    case Shape::Cube:
      return std::abs(z_query - o.center.z()) <= o.half_extents.z();
  }
  return false;
}

// 点到障碍物表面的最小 3D 距离（用于报告里的最小障碍距离曲线）。
// 单位 m。点在内部返回 0。
inline double distanceToObstacle(const Obstacle& o, const Vec3d& p) {
  switch (o.shape) {
    case Shape::Sphere:
      return std::max(0.0, (p - o.center).norm() - o.radius);
    case Shape::Cylinder: {
      double dx = p.x() - o.center.x();
      double dy = p.y() - o.center.y();
      double dxy = std::sqrt(dx * dx + dy * dy);
      double radial = std::max(0.0, dxy - o.radius);
      // z 方向不计入柱高内部，简化：用径向距离
      return radial;
    }
    case Shape::Cube: {
      Vec3d d = (p - o.center).cwiseAbs() - o.half_extents;
      Vec3d clamped = d.cwiseMax(0.0);
      return clamped.norm();
    }
  }
  return 1e9;
}

inline std::string shapeName(Shape s) {
  switch (s) {
    case Shape::Sphere:   return "sphere";
    case Shape::Cylinder: return "cylinder";
    case Shape::Cube:     return "cube";
  }
  return "unknown";
}

}  // namespace drone_common
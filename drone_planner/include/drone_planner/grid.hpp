#pragma once
//
// grid.hpp — 2D 膨胀栅格构建（障碍物 + safety_distance → occupancy）
// 巡航高度 z_cruise：z 跨度不含 z_cruise 的障碍物被忽略（3D 意识）
//

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <algorithm>
#include "drone_common/obstacle.hpp"

namespace drone_planner {

struct Grid2D {
  int nx = 0, ny = 0;
  double resolution = 0.1;
  double x0 = 0, y0 = 0;           // 原点（世界坐标）
  std::vector<uint8_t> cells;       // 0 = free, 1 = occupied

  bool inBounds(int ix, int iy) const {
    return ix >= 0 && ix < nx && iy >= 0 && iy < ny;
  }
  uint8_t& at(int ix, int iy) { return cells[ix + iy * nx]; }
  uint8_t at(int ix, int iy) const { return cells[ix + iy * nx]; }

  void worldToGrid(double x, double y, int& ix, int& iy) const {
    ix = static_cast<int>(std::floor((x - x0) / resolution));
    iy = static_cast<int>(std::floor((y - y0) / resolution));
  }
  void gridToWorld(int ix, int iy, double& x, double& y) const {
    x = x0 + (ix + 0.5) * resolution;  // cell center
    y = y0 + (iy + 0.5) * resolution;
  }
};

// 用障碍物列表构建膨胀栅格
// z_cruise: 当前巡航高度——z 跨度不含该高度的障碍物忽略
inline Grid2D buildOccupancyGrid(
    const std::vector<drone_common::Obstacle>& obstacles,
    double resolution, double x_min, double x_max,
    double y_min, double y_max, double inflate, double z_cruise) {

  Grid2D g;
  g.resolution = resolution;
  g.x0 = x_min; g.y0 = y_min;
  g.nx = static_cast<int>(std::ceil((x_max - x_min) / resolution));
  g.ny = static_cast<int>(std::ceil((y_max - y_min) / resolution));
  g.cells.assign(g.nx * g.ny, 0);

  for (const auto& o : obstacles) {
    // 检查 z 跨度
    if (!drone_common::zOverlap(o, z_cruise)) continue;

    auto f = drone_common::footprint2D(o, inflate);
    // 标记 coverage
    int ix0, iy0, ix1, iy1;
    g.worldToGrid(f.aabb_min.x(), f.aabb_min.y(), ix0, iy0);
    g.worldToGrid(f.aabb_max.x(), f.aabb_max.y(), ix1, iy1);
    ix0 = std::clamp(ix0, 0, g.nx - 1);
    ix1 = std::clamp(ix1, 0, g.nx - 1);
    iy0 = std::clamp(iy0, 0, g.ny - 1);
    iy1 = std::clamp(iy1, 0, g.ny - 1);

    if (f.is_disk) {
      // 圆盘：距离检查更精确
      double cx = o.center.x(), cy = o.center.y();
      double r2 = f.radius * f.radius;
      for (int iy = iy0; iy <= iy1; ++iy) {
        for (int ix = ix0; ix <= ix1; ++ix) {
          double gx, gy;
          g.gridToWorld(ix, iy, gx, gy);
          double dx = gx - cx, dy = gy - cy;
          if (dx * dx + dy * dy <= r2) g.at(ix, iy) = 1;
        }
      }
    } else {
      // AABB：直接占满
      for (int iy = iy0; iy <= iy1; ++iy)
        for (int ix = ix0; ix <= ix1; ++ix)
          g.at(ix, iy) = 1;
    }
  }
  return g;
}

}  // namespace drone_planner
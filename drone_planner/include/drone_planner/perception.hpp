#pragma once
//
// perception.hpp — PerceptionEngine: 四模式路由
//   模式1: GLOBAL + 2D →  委托 buildOccupancyGrid + astarSearch
//   模式2: FOV    + 2D →  queryFOV 投影到 2D 栅格 → astarSearch
//   模式3: GLOBAL + 3D →  全图体素栅格 → astarSearch3D
//   模式4: FOV    + 3D →  queryFOV 过滤不可见体素 → astarSearch3D
//
// FOV 投影规则 (2D): 对每个 2D 网格单元的 z 列 (z_cruise ± drone_half_h),
//   任一可见体素 = 占用 → 该 2D 单元 = 占用
// FOV 过滤规则 (3D): 不可见 = 不安全 = 占用; 只穿过已知自由的体素
//

#include "drone_common/voxel.hpp"
#include "drone_planner/grid.hpp"
#include "drone_planner/astar.hpp"
#include "drone_planner/astar3d.hpp"
#include <string>
#include <vector>
#include <cmath>

namespace drone_planner {

enum class PerceptionMode { GLOBAL = 0, FOV = 1 };
enum class PlannerDim    { TWO_D = 0, THREE_D = 1 };

struct PerceptionConfig {
  PerceptionMode mode = PerceptionMode::GLOBAL;
  PlannerDim dim = PlannerDim::TWO_D;
  double fov_h_rad = 1.047;    // 默认 60°
  double fov_v_rad = 0.785;    // 默认 45°
  double fov_range   = 5.0;    // m
  double voxel_res   = 0.2;    // m
  double z_min       = 0.0;    // 体素栅格 z 下界
  double z_max       = 4.0;    // 体素栅格 z 上界
};

class PerceptionEngine {
 public:
  PerceptionEngine(const PerceptionConfig& cfg) : cfg_(cfg) {}

  // 从障碍物列表重建全图体素栅格（模式 3,4 共用）
  void rebuild(const std::vector<drone_common::Obstacle>& obstacles,
               double inflation) {
    std::vector<double> bounds = {-3.0, -3.0, cfg_.z_min, 5.0, 5.0, cfg_.z_max};
    grid_ = drone_common::VoxelGrid(bounds, cfg_.voxel_res);
    for (const auto& o : obstacles) {
      switch (o.shape) {
        case drone_common::Shape::Sphere:
          grid_.markSphere(o.center, o.radius, inflation);
          break;
        case drone_common::Shape::Cylinder:
          grid_.markCylinder(o.center, o.radius, o.height, inflation);
          break;
        case drone_common::Shape::Cube:
          grid_.markCube(o.center, o.half_extents, inflation);
          break;
      }
    }
  }

  // ====================================================================
  // 2D 模式: 返回 Grid2D（用于 astarSearch）
  // ====================================================================
  Grid2D buildGrid2D(const std::vector<drone_common::Obstacle>& obstacles,
                     double resolution, double x_min, double x_max,
                     double y_min, double y_max, double inflate, double z_cruise,
                     const drone_common::Vec3d& drone_pos,
                     const drone_common::Vec3d& drone_forward) {
    if (cfg_.mode == PerceptionMode::GLOBAL) {
      return buildOccupancyGrid(obstacles, resolution, x_min, x_max,
                                y_min, y_max, inflate, z_cruise);
    }
    // FOV+2D: 从体素栅格 queryFOV，投影到 2D
    return projectFOVto2D(resolution, x_min, x_max, y_min, y_max, z_cruise,
                          drone_pos, drone_forward);
  }

  // ====================================================================
  // 3D 模式: 返回可搜索的体素栅格引用（或 FOV 过滤后的副本）
  // ====================================================================
  const drone_common::VoxelGrid& activeGrid3D(
      const drone_common::Vec3d& drone_pos,
      const drone_common::Vec3d& drone_forward) {
    if (cfg_.mode == PerceptionMode::GLOBAL) return grid_;
    // FOV+3D: 复制 grid_，不可见格标记为占用
    auto fov = grid_.queryFOV(drone_pos, drone_forward,
                              cfg_.fov_h_rad, cfg_.fov_v_rad, cfg_.fov_range);
    fov_grid_ = grid_;
    for (int z = 0; z < fov_grid_.dims().z(); ++z)
      for (int y = 0; y < fov_grid_.dims().y(); ++y)
        for (int x = 0; x < fov_grid_.dims().x(); ++x) {
          size_t idx = x + fov_grid_.dims().x() * (y + fov_grid_.dims().y() * z);
          if (!fov.visible[idx]) fov_grid_.at(x, y, z) = 1;
        }
    return fov_grid_;
  }

  // FOV 查询（给可视化用）
  drone_common::VoxelGrid::FOVResult queryFOV(
      const drone_common::Vec3d& pos, const drone_common::Vec3d& fwd) const {
    return grid_.queryFOV(pos, fwd, cfg_.fov_h_rad, cfg_.fov_v_rad, cfg_.fov_range);
  }

  const drone_common::VoxelGrid& voxelGrid() const { return grid_; }
  PerceptionConfig& config() { return cfg_; }
  const PerceptionConfig& config() const { return cfg_; }

 private:
  // FOV → 2D 投影
  Grid2D projectFOVto2D(double resolution, double x_min, double x_max,
                         double y_min, double y_max, double z_cruise,
                         const drone_common::Vec3d& drone_pos,
                         const drone_common::Vec3d& drone_forward) {
    Grid2D g;
    g.resolution = resolution; g.x0 = x_min; g.y0 = y_min;
    g.nx = static_cast<int>(std::ceil((x_max - x_min) / resolution));
    g.ny = static_cast<int>(std::ceil((y_max - y_min) / resolution));
    g.cells.assign(g.nx * g.ny, 0);

    auto fov = grid_.queryFOV(drone_pos, drone_forward,
                               cfg_.fov_h_rad, cfg_.fov_v_rad, cfg_.fov_range);

    // 对每个 2D 单元格：沿 z 列搜索该体素列是否有可见占用格
    double half_h = 0.25;  // 半机身高度
    int z_lo = static_cast<int>(std::ceil((z_cruise - half_h - cfg_.z_min) / cfg_.voxel_res));
    int z_hi = static_cast<int>(std::ceil((z_cruise + half_h - cfg_.z_min) / cfg_.voxel_res));

    for (int iy = 0; iy < g.ny; ++iy) {
      for (int ix = 0; ix < g.nx; ++ix) {
        // 2D 单元格中心 → 体素坐标范围
        double wx, wy;
        g.gridToWorld(ix, iy, wx, wy);
        drone_common::Vec3d w(wx, wy, z_cruise);
        drone_common::Vec3i gi = grid_.worldToGrid(w);
        for (int z = std::max(0, gi.z() - 2); z <= std::min(grid_.dims().z() - 1, gi.z() + 2); ++z) {
          int gx = gi.x(), gy = gi.y();
          size_t idx = gx + grid_.dims().x() * (gy + grid_.dims().y() * z);
          if (grid_.inBounds(gx, gy, z) && fov.visible[idx] && grid_.at(gx, gy, z) == 1) {
            g.at(ix, iy) = 1;
            break;
          }
        }
      }
    }
    return g;
  }

  PerceptionConfig cfg_;
  drone_common::VoxelGrid grid_{{-3.0, -3.0, 0.0, 5.0, 5.0, 4.0}, 0.2};
  drone_common::VoxelGrid fov_grid_{{-3.0, -3.0, 0.0, 5.0, 5.0, 4.0}, 0.2};
};

}  // namespace drone_planner

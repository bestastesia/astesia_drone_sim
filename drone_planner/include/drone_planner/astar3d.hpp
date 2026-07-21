#pragma once
//
// astar3d.hpp — 3D A* on voxel grid
// 26-connected, 3D octile heuristic, 3D Bresenham smoothing
// 输入: VoxelGrid (cells_ 中 0=free, 1=occupied)
// 输出: vector<Vec3d> 世界坐标路径 (空=无路径)
//

#include "drone_common/voxel.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>
#include <functional>

namespace drone_planner {

using drone_common::Vec3d;
using drone_common::Vec3i;
using drone_common::VoxelGrid;

struct Node3D {
  int ix, iy, iz;
  double g = std::numeric_limits<double>::infinity();
  double h = 0;
  Node3D* parent = nullptr;
  bool closed = false;
  double f() const { return g + h; }
};

// 26连通方向
inline constexpr int kDirs26[26][3] = {
  // 6 面邻居 (cost = 1)
  {1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1},
  // 12 边邻居 (cost = sqrt(2) ≈ 1.414)
  {1,1,0},{1,-1,0},{-1,1,0},{-1,-1,0},
  {1,0,1},{1,0,-1},{-1,0,1},{-1,0,-1},
  {0,1,1},{0,1,-1},{0,-1,1},{0,-1,-1},
  // 8 角邻居 (cost = sqrt(3) ≈ 1.732)
  {1,1,1},{1,1,-1},{1,-1,1},{1,-1,-1},
  {-1,1,1},{-1,1,-1},{-1,-1,1},{-1,-1,-1}
};

// 3D octile heuristic
inline double heuristic3D(int x1, int y1, int z1, int x2, int y2, int z2) {
  int dx = std::abs(x1 - x2);
  int dy = std::abs(y1 - y2);
  int dz = std::abs(z1 - z2);
  int sorts[3] = {dx, dy, dz};
  if (sorts[0] > sorts[1]) std::swap(sorts[0], sorts[1]);
  if (sorts[1] > sorts[2]) std::swap(sorts[1], sorts[2]);
  if (sorts[0] > sorts[1]) std::swap(sorts[0], sorts[1]);
  // d_min(smallest), d_mid, d_max
  return sorts[2] * 1.0 + sorts[1] * (std::sqrt(2.0) - 1.0) + sorts[0] * (std::sqrt(3.0) - std::sqrt(2.0));
}

// 3D Bresenham — 检查直线是否全 free
inline bool lineFree3D(const VoxelGrid& grid, int x0, int y0, int z0, int x1, int y1, int z1) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int dz = std::abs(z1 - z0), sz = z0 < z1 ? 1 : -1;
  int dm = std::max({dx, dy, dz});
  int x = x0, y = y0, z = z0;
  double errx = dm * 0.5, erry = dm * 0.5, errz = dm * 0.5;
  for (int i = 0; i <= dm; ++i) {
    if (i > 0) {  // skip start cell
      if (!grid.inBounds(x, y, z) || grid.at(x, y, z) != 0) return false;
    }
    errx -= dx; erry -= dy; errz -= dz;
    if (errx < 0) { errx += dm; x += sx; }
    if (erry < 0) { erry += dm; y += sy; }
    if (errz < 0) { errz += dm; z += sz; }
  }
  return true;
}

struct TripleHash {
  size_t operator()(const std::tuple<int,int,int>& t) const {
    return static_cast<size_t>(std::get<0>(t))
         ^ (static_cast<size_t>(std::get<1>(t)) << 12)
         ^ (static_cast<size_t>(std::get<2>(t)) << 24);
  }
};

// 返回某 step 的代价
inline double stepCost3D(int d) {
  if (d < 6) return 1.0;          // face
  if (d < 18) return std::sqrt(2.0); // edge
  return std::sqrt(3.0);            // corner
}

// A* 3D 搜索。返回世界坐标路径。grid 中 cells 0=free, 1=occupied。
inline std::vector<Vec3d> astarSearch3D(
    const VoxelGrid& grid, const Vec3d& start, const Vec3d& goal) {

  Vec3i sg = grid.worldToGrid(start);
  Vec3i gg = grid.worldToGrid(goal);
  int sx = sg.x(), sy = sg.y(), sz = sg.z();
  int gx = gg.x(), gy = gg.y(), gz = gg.z();

  // BFS nearest free for start
  if (grid.inBounds(sx, sy, sz) && grid.at(sx, sy, sz)) {
    bool found = false;
    for (int r = 1; r < 20 && !found; ++r) {
      for (int dx = -r; dx <= r && !found; ++dx)
        for (int dy = -r; dy <= r && !found; ++dy)
          for (int dz = -r; dz <= r && !found; ++dz) {
            int cx = sx + dx, cy = sy + dy, cz = sz + dz;
            if (grid.inBounds(cx, cy, cz) && grid.at(cx, cy, cz) == 0) {
              sx = cx; sy = cy; sz = cz; found = true;
            }
          }
    }
    if (!found) return {};
  }
  // BFS nearest free for goal
  if (grid.inBounds(gx, gy, gz) && grid.at(gx, gy, gz)) {
    bool found = false;
    for (int r = 1; r < 20 && !found; ++r) {
      for (int dx = -r; dx <= r && !found; ++dx)
        for (int dy = -r; dy <= r && !found; ++dy)
          for (int dz = -r; dz <= r && !found; ++dz) {
            int cx = gx + dx, cy = gy + dy, cz = gz + dz;
            if (grid.inBounds(cx, cy, cz) && grid.at(cx, cy, cz) == 0) {
              gx = cx; gy = cy; gz = cz; found = true;
            }
          }
    }
    if (!found) return {};
  }
  if (!grid.inBounds(sx, sy, sz) || !grid.inBounds(gx, gy, gz)) return {};

  std::unordered_map<std::tuple<int,int,int>, Node3D, TripleHash> nodes;
  auto cmp = [](Node3D* a, Node3D* b) { return a->f() > b->f(); };
  std::priority_queue<Node3D*, std::vector<Node3D*>, decltype(cmp)> open(cmp);

  auto& sn = nodes[{sx, sy, sz}];
  sn.ix = sx; sn.iy = sy; sn.iz = sz; sn.g = 0;
  sn.h = heuristic3D(sx, sy, sz, gx, gy, gz);
  open.push(&sn);

  bool found = false;
  while (!open.empty()) {
    Node3D* cur = open.top(); open.pop();
    if (cur->closed) continue;
    cur->closed = true;
    if (cur->ix == gx && cur->iy == gy && cur->iz == gz) { found = true; break; }
    for (int d = 0; d < 26; ++d) {
      int nx = cur->ix + kDirs26[d][0];
      int ny = cur->iy + kDirs26[d][1];
      int nz = cur->iz + kDirs26[d][2];
      if (!grid.inBounds(nx, ny, nz) || grid.at(nx, ny, nz) != 0) continue;
      double ng = cur->g + stepCost3D(d);
      auto& nb = nodes[{nx, ny, nz}];
      nb.ix = nx; nb.iy = ny; nb.iz = nz;
      if (ng < nb.g) {
        nb.g = ng; nb.h = heuristic3D(nx, ny, nz, gx, gy, gz);
        nb.parent = cur;
        open.push(&nb);
      }
    }
  }

  if (!found) return {};

  // backtrack
  std::vector<Vec3d> path;
  Node3D* p = &nodes[{gx, gy, gz}];
  while (p) {
    path.push_back(grid.gridToWorld(Vec3i(p->ix, p->iy, p->iz)));
    p = p->parent;
  }
  std::reverse(path.begin(), path.end());

  // 3D Bresenham 碰撞感知平滑
  std::vector<Vec3d> smooth;
  smooth.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const auto& a = smooth.back();
    const auto& c = path[i + 1];
    Vec3i ag = grid.worldToGrid(a);
    Vec3i cg = grid.worldToGrid(c);
    if (lineFree3D(grid, ag.x(), ag.y(), ag.z(), cg.x(), cg.y(), cg.z())) continue;
    smooth.push_back(path[i]);
  }
  smooth.push_back(path.back());

  if (smooth.size() <= 2) return path;  // 平滑坍塌 → 保留原始
  return smooth;
}

}  // namespace drone_planner

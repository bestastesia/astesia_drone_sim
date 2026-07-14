#pragma once
//
// astar.hpp — A* on 2D inflated occupancy grid
// 8-connected, octile heuristic, collinearity reduction smoothing
//

#include "grid.hpp"
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <limits>

namespace drone_planner {

struct Node {
  int ix, iy;
  double g = std::numeric_limits<double>::infinity();
  double h = 0;
  Node* parent = nullptr;
  bool closed = false;

  double f() const { return g + h; }
};

// 8 连通方向
constexpr int kDirs8[8][2] = {
  {-1,-1}, {0,-1}, {1,-1}, {-1,0}, {1,0}, {-1,1}, {0,1}, {1,1}
};

// octile heuristic
inline double heuristic(int x1, int y1, int x2, int y2) {
  int dx = std::abs(x1 - x2), dy = std::abs(y1 - y2);
  return 1.0 * std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
}

// 哈希：pair<int,int> → size_t
struct PairHash {
  size_t operator()(const std::pair<int,int>& p) const {
    return static_cast<size_t>(p.first) ^ (static_cast<size_t>(p.second) << 16);
  }
};

// A* 搜索。返回格子路径（世界坐标），空 = 无路径
inline std::vector<Eigen::Vector2d> astarSearch(
    const Grid2D& grid, double start_x, double start_y,
    double goal_x, double goal_y) {

  int sx, sy, gx, gy;
  grid.worldToGrid(start_x, start_y, sx, sy);
  grid.worldToGrid(goal_x, goal_y, gx, gy);

  // start/goal 在占用格内 → 从最近 free cell 扩
  if (grid.inBounds(sx, sy) && grid.at(sx, sy)) {
    // 简单 BFS 找最近 free
    bool found = false;
    for (int r = 1; r < 20 && !found; ++r) {
      for (int d = -r; d <= r && !found; ++d) {
        int cx = sx + d, cy = sy - r;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { sx = cx; sy = cy; found = true; }
        cy = sy + r;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { sx = cx; sy = cy; found = true; }
        cx = sx - r; cy = sy + d;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { sx = cx; sy = cy; found = true; }
        cx = sx + r;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { sx = cx; sy = cy; found = true; }
      }
    }
    if (!found) return {};
  }
  if (grid.inBounds(gx, gy) && grid.at(gx, gy)) {
    bool found = false;
    for (int r = 1; r < 20 && !found; ++r) {
      for (int d = -r; d <= r && !found; ++d) {
        int cx = gx + d, cy = gy - r;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { gx = cx; gy = cy; found = true; }
        cy = gy + r;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { gx = cx; gy = cy; found = true; }
        cx = gx - r; cy = gy + d;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { gx = cx; gy = cy; found = true; }
        cx = gx + r;
        if (grid.inBounds(cx, cy) && grid.at(cx, cy) == 0) { gx = cx; gy = cy; found = true; }
      }
    }
    if (!found) return {};
  }
  if (!grid.inBounds(sx, sy) || !grid.inBounds(gx, gy)) return {};

  std::unordered_map<std::pair<int,int>, Node, PairHash> nodes;
  auto cmp = [](Node* a, Node* b) { return a->f() > b->f(); };
  std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);

  auto& sn = nodes[{sx, sy}];
  sn.ix = sx; sn.iy = sy; sn.g = 0; sn.h = heuristic(sx, sy, gx, gy);
  open.push(&sn);

  bool found = false;
  while (!open.empty()) {
    Node* cur = open.top(); open.pop();
    if (cur->closed) continue;
    cur->closed = true;
    if (cur->ix == gx && cur->iy == gy) { found = true; break; }
    for (int d = 0; d < 8; ++d) {
      int nx = cur->ix + kDirs8[d][0], ny = cur->iy + kDirs8[d][1];
      if (!grid.inBounds(nx, ny) || grid.at(nx, ny) != 0) continue;
      double step_cost = (d % 2 == 0) ? 1.414 : 1.0;  // diag 1.414
      double ng = cur->g + step_cost;
      auto& nb = nodes[{nx, ny}];
      nb.ix = nx; nb.iy = ny;
      if (ng < nb.g) {
        nb.g = ng; nb.h = heuristic(nx, ny, gx, gy);
        nb.parent = cur;
        open.push(&nb);
      }
    }
  }

  if (!found) return {};

  // 回溯 + 转世界坐标
  std::vector<Eigen::Vector2d> path;
  Node* p = &nodes[{gx, gy}];
  while (p) {
    double wx, wy;
    grid.gridToWorld(p->ix, p->iy, wx, wy);
    path.emplace_back(wx, wy);
    p = p->parent;
  }
  std::reverse(path.begin(), path.end());

  // 共线点简化
  std::vector<Eigen::Vector2d> smooth;
  smooth.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const auto& a = smooth.back();
    const auto& b = path[i];
    const auto& c = path[i + 1];
    // 检查 b 是否在 ac 线段上（容许一个格子宽度误差）
    double cross = (b.x() - a.x()) * (c.y() - a.y()) - (b.y() - a.y()) * (c.x() - a.x());
    double dot = (b.x() - a.x()) * (c.x() - a.x()) + (b.y() - a.y()) * (c.y() - a.y());
    double d_ac = (c - a).norm();
    if (std::abs(cross) < 1e-6 && dot > 0 && dot < d_ac * d_ac + 0.1) continue;
    smooth.push_back(b);
  }
  smooth.push_back(path.back());

  return smooth;
}

}  // namespace drone_planner
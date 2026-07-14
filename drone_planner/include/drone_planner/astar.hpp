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

// Bresenham 直线从 (x0,y0) 到 (x1,y1) 的各格子是否全是 free
inline bool lineFree(const Grid2D& grid, int x0, int y0, int x1, int y1) {
  int dx = std::abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -std::abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  int x = x0, y = y0;
  while (true) {
    if (x != x0 || y != y0) {  // 跳过起点本身
      if (!grid.inBounds(x, y) || grid.at(x, y) != 0) return false;
    }
    if (x == x1 && y == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x += sx; }
    if (e2 <= dx) { err += dx; y += sy; }
  }
  return true;
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

  // 碰撞感知平滑：只有直线段全程 free 时才跳掉中间点
  std::vector<Eigen::Vector2d> smooth;
  smooth.push_back(path.front());
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const auto& a = smooth.back();
    const auto& c = path[i + 1];
    int ax, ay, cx, cy;
    grid.worldToGrid(a.x(), a.y(), ax, ay);
    grid.worldToGrid(c.x(), c.y(), cx, cy);
    int bx, by;
    grid.worldToGrid(path[i].x(), path[i].y(), bx, by);
    // 如果 a→c 直线在栅格上无碰撞，跳过中间点 b
    if (lineFree(grid, ax, ay, cx, cy)) continue;
    smooth.push_back(path[i]);
  }
  smooth.push_back(path.back());

  return smooth;
}

}  // namespace drone_planner
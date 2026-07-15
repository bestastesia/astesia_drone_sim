# asteesia_drone_sim

ROS2 Humble 四旋翼无人机仿真器——2027 保研招生编程题（60 分）。

**如果你从没见过这个项目，这份文档就是写给你的。** 读完你能在 5 分钟内构建、运行、发目标、改参数、录数据。

---

## 目录
1. [系统要求](#系统要求)
2. [克隆与构建](#克隆与构建)
3. [一键启动](#一键启动)
4. [怎么让无人机飞到指定位置](#怎么让无人机飞到指定位置)
5. [地图与障碍物](#地图与障碍物)
6. [控制器调参](#控制器调参)
7. [规划器参数](#规划器参数)
8. [录制实验数据](#录制实验数据)
9. [系统架构](#系统架构)
10. [实时监控仪表盘](#实时监控仪表盘)
11. [常见故障排查](#常见故障排查)

---
## 目录

1. [系统要求](#系统要求)
2. [克隆与构建](#克隆与构建)
3. [一键启动](#一键启动)
4. [怎么让无人机飞到指定位置](#怎么让无人机飞到指定位置)
5. [地图与障碍物](#地图与障碍物)
6. [控制器调参](#控制器调参)
7. [规划器参数](#规划器参数)
8. [录制实验数据](#录制实验数据)
9. [系统架构](#系统架构)
10. [目录结构](#目录结构)
11. [实时监控仪表盘](#实时监控仪表盘)
12. [常见故障排查](#常见故障排查)

---

## 系统要求

| 依赖 | 版本 |
|---|---|
| 操作系统 | Ubuntu 22.04 |
| ROS2 | Humble (`apt install ros-humble-desktop`) |
| Eigen | 3.4 (`apt install libeigen3-dev`) |
| colcon | `apt install python3-colcon-common-extensions` |
| C++ 编译器 | g++ 11+ (C++17) |
| git | 任意 |

**特别注意**：如果你装了 Anaconda/Miniconda，必须看 [常见故障 §1](#1-colcon-build-报错-no-module-named-catkin_pkg)。

---

## 克隆与构建

```bash
# 1. 创建工作区
mkdir -p ~/drone_sim_ws/src
cd ~/drone_sim_ws

# 2. 克隆代码
git clone git@github.com:bestastesia/astesia_drone_sim.git src/astesia_drone_sim

# 3. 构建（注意 --cmake-args 那串——conda 用户必须带）
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3

# 4. 加载环境
source install/setup.bash
```

构建成功后你应该看到 `Summary: 6 packages finished`。

---

## 一键启动

### 基础模式（动力学 + 控制器 + 地图 + RViz，无避障规划器）

```bash
ros2 launch drone_bringup no_planner.launch.py
```

适用于：悬停、目标点、正方形航线。

### 完整模式（+ 避障规划器）

```bash
ros2 launch drone_bringup full.launch.py
```

适用于：静态避障、狭窄通道绕行。控制器从规划器接收安全目标点，自动绕开障碍物。

---

## 怎么让无人机飞到指定位置

### 方法 1：RViz 里手动点（最简单）

1. RViz 窗口顶部工具栏，点 **2D Goal Pose**（旗子图标）
2. 在 `map` 窗口点一个位置
3. 弹出框里 **把 z 改成 1.5**，点确定
4. 无人机开始飞过去

### 方法 2：命令行发目标

```bash
# 飞到 (2, 1, 1.5)
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}}}'
```

把 `x`, `y`, `z` 改成你想要的——这就是告诉无人机「飞到这里」。

### 方法 3：跑正方形航线（多目标点自动流转）

```bash
ros2 run drone_bringup waypoint_sender.py --ros-args -p hold_time:=4.0
```

默认走 5 个点：(0,0,1.5) → (2,0,1.5) → (2,2,1.5) → (0,2,1.5) → (0,0,1.5)，每个点停 4 秒。  
改航线：`-p waypoints:="[[0,0,1.5],[3,0,1.5],[3,3,1.5],[0,3,1.5]]"`  
改停留时间：`-p hold_time:=6.0`

---

## 地图与障碍物

### 默认 6 个障碍物（显式 YAML 配置）

`src/astesia_drone_sim/drone_map/config/map.yaml`:
```yaml
drone_map:
  ros__parameters:
    procedural: false       # false = 用手写的障碍物列表
    seed: 42
    obstacles:
      - "sphere 0.7 0.3 1.5 0.35"       # 球 cx cy cz radius
      - "sphere 1.0 0.5 1.5 0.35"
      - "sphere 1.3 0.5 1.5 0.2"
      - "cylinder 0.5 0.7 1.0 0.25 2.0" # 柱 cx cy cz radius height
      - "cube 0.8 -0.2 1.5 0.2 0.2 0.3" # 立方 cx cy cz hx hy hz
      - "sphere 0.4 0.0 1.5 0.3"
```

**改障碍物**：编辑这个 YAML → 保存 → Ctrl+C 停掉 launch → 重新 `ros2 launch`。

### 程序随机生成（固定种子，可复现）

编辑 `map.yaml`，把 `procedural: false` 改成 `procedural: true`，然后改这些参数：

| 参数 | 默认 | 含义 |
|---|---|---|
| `seed` | 42 | 随机种子（相同 seed = 相同地图） |
| `num_obstacles` | 8 | 障碍物数量 |
| `bounds` | [-2, 4, -2, 4, 0, 5] | 生成区域 [x0 x1 y0 y1 z0 z1] |
| `clear_radius` | 0.8 | 障碍物离起点和终点的最小距离 |
| `size_min` / `size_max` | 0.2 / 0.5 | 障碍物尺寸范围 |

### 运行时动态切换（不用重启）

```bash
# 先把当前模式改成 procedural
ros2 param set /drone_map procedural true
# 改 seed 换一组随机障碍物
ros2 param set /drone_map seed 77
ros2 param set /drone_map num_obstacles 10
# 障碍物即时更新，RViz 里立即看到变化
```

**注意**：如果系统是通过 `ros2 launch` 启动的，`ros2 param set` 直接作用在正在运行的节点上——不需要 `pkill` 再 `ros2 run`。`ros2 run` 是**单独**启动一个新节点，会和 launch 已有的节点冲突。

---

## 控制器调参

所有参数在 `src/astesia_drone_sim/drone_controller/config/controller.yaml`。

### 位置 PD 参数

| 参数 | 默认值 | 含义 | 调大→ |
|---|---|---|---|
| `Kp_pos` | [2.0, 2.0, 3.0] | [x, y, z] 位置 P | 更硬地拉向目标 |
| `Kd_pos` | [2.0, 2.0, 2.4] | [x, y, z] 位置 D | 刹得更快，但太大会抖 |
| `a_xy_max` | 4.0 | 水平加速度上限 m/s² | 允许更猛的倾斜 |
| `a_z_max` | 6.0 | 上升加速度上限 | 更快爬升 |
| `a_z_min` | -3.0 | 下降加速度上限 | 更快下降 |

**阻尼比公式**：`ζ = Kd / (2 × √Kp)`。ζ < 1 有超调，ζ > 1 过阻尼（平滑但慢）。

### 姿态参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `Kp_att` | [8, 8, 3] | [roll, pitch, yaw] 姿态 P |
| `Kd_rate` | [0.8, 0.8, 0.7] | 角速度 D |

### 物理参数（必须和动力学节点一致）

| 参数 | 默认值 | 含义 |
|---|---|---|
| `mass` | 1.0 | 质量 kg |
| `k_F` | 1.0 | 推力系数 N·s²/rad² |
| `k_M` | 0.05 | 反扭矩系数 N·m·s²/rad² |
| `arm_length` | 0.2 | 臂长 m |
| `rpm_min` / `rpm_max` | 0 / 10000 | RPM 限幅 |
| `F_max` | 39.24 | 最大总推力 N（= 4×kF×ω_max²） |

**改参后重启 launch 即生效**，不需要 colcon build。

---

## 规划器参数

在 `src/astesia_drone_sim/drone_planner/config/planner.yaml`。

| 参数 | 默认值 | 含义 |
|---|---|---|
| `resolution` | 0.1 | 栅格分辨率 m（越小路径越精细但越慢） |
| `bounds` | [-3,5,-3,5] | 地图区域 [x0,x1,y0,y1] |
| `safety_distance` | 0.4 | 障碍物安全距离 m |
| `drone_radius` | 0.2 | 无人机外接圆 m |
| `replan_rate` | 1.0 | 重规划频率 Hz |
| `advance_tol` | 0.15 | 到达航点的距离阈值 m |
| `use_goal_z` | true | true=按目标的 z 高度规划；false=固定巡航高度 |

---

## 录制实验数据

每个验收场景一条命令，自动录 bag + 打印数值摘要。

```bash
# 先启动系统（终端 1）
ros2 launch drone_bringup no_planner.launch.py

# 终端 2 —— 录悬停 8 秒
bash src/astesia_drone_sim/drone_bringup/scripts/record_and_analyze.sh bag_hover 8
```

**输出例**：
```
Samples: 353
Steady-state z error (last 20%): 0.0001m
Last z: 1.5001
Max z: 1.5116 (overshoot: 0.0116m)
```

**完整验收录制流程**见 `report/verification_guide.html`——浏览器打开，每个场景都有复制即用的命令。

---

## 系统架构

### 节点拓扑（箭头 = ROS2 topic）

```
  ┌──────────┐    MarkerArray     ┌──────────┐
  │ drone_map │ ─────────────────→│ planner  │
  └──────────┘                   │ A* grid  │──→ safe_goal (PoseStamped)
       ↑                         └────┬─────┘
       │ obstacles                     │
       │                         ┌─────┴──────┐
       │                         │ controller │──→ rpm_cmd (Float32MultiArray[4])
       │                         │ PD + mixer │
       │                         └─────┬──────┘
       │                               │
  ┌────┴───────────────────────────────┴────┐
  │            drone_dynamics               │
  │  EOM + motor 1st-order + quaternion     │
  └────────────────┬────────────────────────┘
                   │ odom / imu / path / tf
              ┌────┴─────┐
              │   RViz2   │
              └──────────┘
```

### 全部话题

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/drone/motor_rpm_cmd` | Float32MultiArray [FL,FR,BL,BR] | controller | dynamics |
| `/drone/odom` | nav_msgs/Odometry | dynamics | controller, planner, rviz |
| `/drone/imu` | sensor_msgs/Imu | dynamics | rviz |
| `/drone/path` | nav_msgs/Path | dynamics | rviz |
| `/tf` (map→base_link) | tf2_msgs/TFMessage | dynamics | rviz |
| `/drone/goal` | geometry_msgs/PoseStamped | rviz 2D Goal / CLI | planner |
| `/drone/safe_goal` | geometry_msgs/PoseStamped | planner | controller |
| `/map/obstacles` | visualization_msgs/MarkerArray | map | planner, rviz |
| `/drone/planned_path` | nav_msgs/Path | planner | rviz |
| `/drone/planner_status` | std_msgs/String | planner | rviz |
| `/drone/drone_marker` | visualization_msgs/MarkerArray | dynamics | rviz |

### tf 树

```
map (fixed frame)
 └── base_link (100Hz, dynamics)
      └── imu_link (static identity)
```

### RViz 里各 display 的含义

| Display | Topic | 颜色 | 含义 |
|---|---|---|---|
| Drone Marker | `/drone/drone_marker` | 绿箭头 | 无人机当前位置+朝向 |
| Odometry | `/drone/odom` | — | 位置+速度向量 |
| Path (history) | `/drone/path` | 绿线 | 历史飞行轨迹 |
| Path (planned) | `/drone/planned_path` | 黄线 | 规划器的避障路径 |
| Obstacles | `/map/obstacles` | 橙色半透明 | 障碍物 |
| Goal | `/drone/goal` | 红点 | 用户设置的目标点 |
| Safe Goal | `/drone/safe_goal` | 蓝点 | 规划器输出的安全子目标 |
| TF | — | 彩色轴 | 坐标系（红=x 前, 绿=y 左, 蓝=z 上） |

---

## 目录结构

```
src/astesia_drone_sim/
├── drone_common/               # 共享头文件（不依赖 ROS）
│   ├── include/drone_common/
│   │   ├── frames.hpp          #   坐标系 + 电机布局（所有节点的单信源）
│   │   ├── mixer.hpp           #   控制分配矩阵 B / B⁻¹
│   │   ├── math.hpp            #   四元数积分 / skew / vee
│   │   ├── obstacle.hpp        #   障碍物结构 + 碰撞检测
│   │   └── drone_common.hpp    #   聚合 include
│   └── test/test_common.cpp    #   7 条 gtest 回归网
├── drone_dynamics/             # 动力学节点（1kHz）
│   ├── include/drone_dynamics/dynamics.hpp
│   ├── src/dynamics_node.cpp
│   └── config/dynamics.yaml
├── drone_controller/           # 控制器节点（200Hz）
│   ├── include/drone_controller/
│   │   ├── controller.hpp      #   级联 PD + 几何姿态
│   │   └── ladrc.hpp           #   LADRC（实验性）
│   ├── src/controller_node.cpp
│   └── config/controller.yaml
├── drone_map/                  # 地图节点（1Hz）
│   ├── src/map_node.cpp
│   └── config/map.yaml
├── drone_planner/              # 规划器节点（1Hz 重规划）
│   ├── include/drone_planner/
│   │   ├── astar.hpp           #   A* 8 连通 + 碰撞感知平滑
│   │   └── grid.hpp            #   2D 膨胀栅格
│   ├── src/planner_node.cpp
│   └── config/planner.yaml
├── drone_bringup/              # 启动与工具
│   ├── launch/
│   │   ├── full.launch.py      #   完整系统（含规划器）
│   │   └── no_planner.launch.py #  基础系统（无规划器）
│   ├── config/（空——各节点 config 在自己包内）
│   ├── rviz/drone.rviz         #   RViz 预设配置
│   ├── scripts/
│   │   ├── waypoint_sender.py  #   多航点顺序发送
│   │   │   ├── record_and_analyze.sh # 录 bag + 自动分析
│   │   │   ├── plot_bag.py         #   bag → CSV + 数值摘要
│   │   │   └── live_monitor.py     #   浏览器实时监控仪表盘
│   └── report/
│       ├── report.md           #   技术报告
│       ├── verification_guide.html # 验收场景录制指南
│       └── ai_usage.md         #   AI 使用说明
├── ai_usage.md                 # AI 辅助编程说明
├── README.md                   # 本文件
└── .gitignore
```

---

## 实时监控仪表盘

浏览器仪表盘，实时显示位置误差、RPM、轨迹、障碍物距离，无需额外依赖。

### 启动

```bash
# 终端 1：仿真栈
ros2 launch drone_bringup full.launch.py

# 终端 2：监控器（⚠️ 不能用 conda 的 python3，必须 source ROS 环境）
source /opt/ros/humble/setup.bash
source install/setup.bash
/usr/bin/python3 src/astesia_drone_sim/drone_bringup/scripts/live_monitor.py
```

浏览器打开 **http://localhost:8765**。

### 四个图表

| 图表 | 内容 | 验收标准 |
|---|---|---|
| 1. Position Error | |x-goal_x| (绿), |z-goal_z| (蓝), 参考线 0.3m | 全部 < 0.3m |
| 2. Motor RPM | 四轴 RPM（自动缩放）, FL/FR/BL/BR | 悬停时稳定在 ~15 RPM |
| 3. XY Trajectory | 无人机实际轨迹（绿线）+ 障碍物（彩色圆圈）+ 目标点（蓝点）| 轨迹不穿透障碍物 |
| 4. Obstacle Distances | 各障碍物距离（彩色细线）+ 最近距离（绿粗线）+ 安全线 0.4m（红虚线）| 最近距离 > 0.4m |

### 状态卡片

- **RPM Sat**: 正常时显示 `OK`（绿），RPM 达到上限时显示 `SATURATED`（红）
- **Att Diverge**: 正常时显示 `OK`（绿），位置/速度 NaN 时显示 `DIVERGED`（红）

### 退出

`Ctrl+C` 退出，自动保存 `dashboard_log.csv`（含全部时间序列数据，可用 Excel/Python 二次分析）。

---

## 常见故障排查

### 1. `colcon build` 报错 `No module named 'catkin_pkg'`

**原因**：Anaconda/Miniconda 的 Python 3.11 劫持了 cmake。

**修复**：**每次** colcon build 必须带完整参数：
```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
把这条写成 alias 方便用：
```bash
alias cb='colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
```

### 2. `ros2 param set` 报 `rclpy.ok()` 错误

**原因**：同上——conda 的 Python 破坏了 ROS2 CLI。

**修复**：运行 `ros2` 命令前先 `unset CONDA_PREFIX`，或者开一个新终端，不要 `conda activate`。

### 3. 无人机不动 / 悬停后直接掉到地上

**排查步骤**：
1. `ros2 topic list | grep drone` —— 确认话题都在
2. `ros2 topic echo /drone/motor_rpm_cmd` —— 应该能看到非零 RPM
3. 如果 RPM 全是 0：控制器没收到目标。发一个目标点试试
4. 如果 `pos= err=` 不停变大：控制器在追目标但追不上→可能是 Kp 太小或 F_max 不够

### 4. RViz 里什么都看不到

**排查步骤**：
1. RViz 左上角 **Fixed Frame** 改成 `map`
2. 点左下 **Add** → 选 **TF** → OK
3. 如果 TF 也看不到：`ros2 topic echo /tf` 确认 dynamics 在发
4. 如果 `/tf` 有消息：检查 RViz 的 Global Options → Fixed Frame = `map`

### 5. 黄色障碍物不出现 / 规划路径是直线（不绕行）

**排查**：
1. `ros2 topic echo /map/obstacles` → 应该看到多个 marker
2. 如果只有 1 个：检查 `drone_map/config/map.yaml` 的 `procedural: false` 和 `obstacles:` 列表
3. `ros2 param set /drone_map procedural true` 切到随机生成看是否生效

### 6. 僵尸进程（之前的节点没杀干净）

```bash
# 一键清掉本项目所有节点
pkill -9 dynamics_node controller_node map_node planner_node rviz2 ros2
```

### 7. `ros2 launch` 启动后 controller 报 `mode=xxx` 但 LADRC 发散

controller 默认 `mode=pd`（稳定的 PD 控制）。如果之前切过 LADRC：
```bash
ros2 param set /drone_controller control_mode pd
```
LADRC 是实验性 bonus，不建议作为默认。

---

## 运行单元测试

```bash
colcon test --packages-select drone_common --event-handlers console_direct+
```
应看到 `[PASSED] 7 tests`——验证 mixer 矩阵、四元数积分、roll/yaw 符号正确。

---

## 更多信息

- **技术报告**: `report/report.md`
- **验收场景录制**: 浏览器打开 `report/verification_guide.html`
- **AI 使用说明**: `ai_usage.md`
- **GitHub**: https://github.com/bestastesia/astesia_drone_sim
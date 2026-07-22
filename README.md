# astesia_drone_sim

ROS2 Humble 四旋翼无人机仿真器 — 2027 保研招生编程题（60 分）。

**如果你从没见过这个项目，这份文档就是写给你的。** 读完你能在 5 分钟内构建、运行、发目标、改参数、录数据。

---

## 目录

1. [系统要求](#系统要求)
2. [克隆与构建](#克隆与构建)
3. [一键启动](#一键启动)
4. [怎么让无人机飞到指定位置](#怎么让无人机飞到指定位置)
5. [轨迹生成器](#轨迹生成器)
6. [系统架构](#系统架构)
7. [控制器设计](#控制器设计)
8. [感知与规划（四模式）](#感知与规划四模式)
9. [地图与障碍物](#地图与障碍物)
10. [风扰与传感器噪声](#风扰与传感器噪声)
11. [实时监控仪表盘（地面站）](#实时监控仪表盘地面站)
12. [录制实验数据](#录制实验数据)
13. [自动验收脚本](#自动验收脚本)
14. [多无人机](#多无人机)
15. [运行单元测试](#运行单元测试)
16. [目录结构](#目录结构)
17. [常见故障排查](#常见故障排查)
18. [更多信息](#更多信息)

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

适用于：悬停、目标点、正方形航线、轨迹跟踪。控制器直接从 `/drone/goal` 取目标（launch 自动 remap `safe_goal → goal`）。

### 完整模式（+ 四模式避障规划器）

```bash
ros2 launch drone_bringup full.launch.py
```

适用于：静态避障、狭窄通道绕行、FOV 局部感知。控制器从规划器接收安全目标点（`/drone/safe_goal`），自动绕开障碍物。

两种 launch 都会自动清理僵尸进程（防止多次 Ctrl+C 后进程叠加）。

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

## 轨迹生成器

参数化轨迹生成器，支持圆形、八字和航点列表，逐帧发布动态目标点。

```bash
# 圆形轨迹（半径 1m, 周期 10s）
ros2 run drone_bringup trajectory_generator.py --ros-args -p type:=circle -p radius:=1.0 -p period:=10.0

# 八字轨迹（Bernoulli lemniscate）
ros2 run drone_bringup trajectory_generator.py --ros-args -p type:=lemniscate -p scale:=1.5 -p period:=15.0

# 航点文件
ros2 run drone_bringup trajectory_generator.py --ros-args -p type:=waypoints -p waypoint_file:=path/to/waypoints.json
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `type` | circle | `circle` / `lemniscate` / `waypoints` |
| `rate` | 10.0 | 目标发布频率 Hz |
| `period` | 10.0 | 轨迹一圈用时 s |
| `radius` | 1.0 | 圆形半径 m |
| `scale` | 1.5 | 八字尺度 m |
| `cx/cy/cz` | 0/0/1.5 | 轨迹中心 |
| `hold_time` | 3.0 | waypoint 模式每点停留时间 s |

**注意**：轨迹生成器发布 `/drone/goal`，与规划器的 `safe_goal` 冲突。使用轨迹时请用 `no_planner.launch.py` 启动。

---

## 系统架构

### 节点拓扑（箭头 = ROS2 topic）

```
                          ┌──────────────┐
                          │   RViz2      │  ← 可视化
                          └──────┬───────┘
                                 │ odom / imu / path / safe_goal / obstacles / tf
                                 │ planned_path / fov_cone / fov_voxels
            ┌────────┐    ┌──────┴───────┐    ┌──────────┐
            │ drone_ │    │   drone_      │    │  drone_  │
            │  map   │───→│  planner     │───→│controller│
            │ Marker │    │ A*/A*3D+FOV │    │ PD+mixer │
            │ Array  │    └──────┬───────┘    └────┬─────┘
            └────────┘           │ safe_goal       │ rpm_cmd
                                 │                 │
                            ┌────┴─────────────────┴────┐
                            │        drone_dynamics      │
                            │   EOM + motor 1st-order    │
                            │   + wind + IMU noise       │
                            └───────────┬────────────────┘
                                        │ odom / imu / path / tf
                                        │
                                  ┌─────┴──────┐
                                  │   RViz2     │
                                  └────────────┘
```

### 全部话题

| 话题 | 类型 | 发布者 | 订阅者 |
|---|---|---|---|
| `/drone/motor_rpm_cmd` | Float32MultiArray [FL,FR,BL,BR] | controller | dynamics |
| `/drone/odom` | nav_msgs/Odometry | dynamics | controller, planner, rviz |
| `/drone/imu` | sensor_msgs/Imu | dynamics | rviz |
| `/drone/path` | nav_msgs/Path (历史轨迹) | dynamics | rviz |
| `/tf` (map→base_link) | tf2_msgs/TFMessage | dynamics | rviz |
| `/drone/goal` | geometry_msgs/PoseStamped | rviz 2D Goal / CLI / 地面站 | planner |
| `/drone/safe_goal` | geometry_msgs/PoseStamped | planner | controller |
| `/drone/planned_path` | nav_msgs/Path (规划路径) | planner | rviz |
| `/drone/planner_status` | std_msgs/String | planner | rviz, 地面站 |
| `/map/obstacles` | visualization_msgs/MarkerArray | map | planner, rviz |
| `/drone/drone_marker` | visualization_msgs/MarkerArray | dynamics | rviz |
| `/drone/fov_cone` | visualization_msgs/Marker (LINE_LIST) | planner | rviz |
| `/drone/fov_voxels` | visualization_msgs/MarkerArray (CUBE_LIST) | planner | rviz |

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
| Odometry | `/drone/odom` | — | 位置+速度向量（ODO 文字） |
| Path (history) | `/drone/path` | 绿线 | 历史飞行轨迹 |
| Path (planned) | `/drone/planned_path` | 黄线 | 规划器的避障路径 |
| Obstacles | `/map/obstacles` | 橙色半透明 | 障碍物 |
| Goal | `/drone/goal` | 红点 | 用户设置的目标点 |
| Safe Goal | `/drone/safe_goal` | 蓝点 | 规划器输出的安全子目标 |
| FOV Cone | `/drone/fov_cone` | 青色线框 | FOV 感知锥体（FOV 模式） |
| FOV Voxels | `/drone/fov_voxels` | 绿色方块 | FOV 内可见障碍物体素（FOV 模式） |
| TF | — | 彩色轴 | 坐标系（红=x 前, 绿=y 左, 蓝=z 上） |

---

## 控制器设计

### 位置环（线性 PD + 有限积分 + 反饱和）

```
e_p = p_goal − p,   e_v = −v (v_des=0)
a_des = Kp·e_p + Kd·e_v + integral
```

- **PD 增益**：Kp=[2.0, 2.0, 3.0], Kd=[2.0, 2.0, 2.4]，按 `ζ=Kd/(2√Kp)≈0.7`（二阶系统阻尼比）配置
- **积分项**：内置 0.02m 死区 + 遇限削弱（conditional anti-windup），仅 x 轴启用（Ki=0.5）以补偿稳态风扰
- **加速度限幅**：a_xy≤4 m/s², a_z∈[-3, 6] m/s², ‖a‖≤8 m/s²

### 姿态环（Lee 几何误差，无 Euler 奇异）

- 推力矢量 `T_W = m(a_des + g)` → `z_B_des = normalize(T_W)`
- `R_des` 由 `z_B_des + yaw` 构造（世界系投影法，避免 gimbal lock）
- 姿态误差 `e_R = ½ vee(R_desᵀR − RᵀR_des)`
- 力矩 `τ = −Kp_R·e_R − Kd_ω·e_ω`
- Kp_att=[8,8,3], Kd_rate=[0.8,0.8,0.7]

### Mixer 逆

`u = B⁻¹·[F, τx, τy, τz]ᵀ` → `ω_i² = clamp(u_i, 0, ω_max²)` → `RPM = ω×60/2π`

B 矩阵在 `drone_common/mixer.hpp` 定义，controller 与 dynamics 共享同一份（single source of truth）。Mixer 符号用 gtest 验证：B·B⁻¹≈I、+roll→左电机快、+yaw→CCW 电机快。

### LADRC（实验性 bonus）

`controller.yaml` 中 `control_mode: "ladrc"` 可切换到二阶线性自抗扰控制。每轴独立 LESO（线性扩张状态观测器）估计总扰动并实时补偿。默认 `control_mode: "pd"`（推荐）。

### 控制器调参

所有参数在 `drone_controller/config/controller.yaml`。**改 YAML 后重启 launch 即生效**，不需重新 build。部分参数支持 `ros2 param set` 热更新（标注 🔥）。详见 `drone_bringup/config/parameter_reference.yaml`。

---

## 感知与规划（四模式）

规划器内置四种感知/规划模式，通过地面站或 `ros2 param set` 实时切换：

```
              全局感知 (GLOBAL)           局部感知 (FOV)
           ┌─────────────────────┐  ┌──────────────────────┐
  2D A*    │ 模式1: 全局 2D       │  │ 模式2: FOV→2D 栅格投影 │
           │ 8连通 A* + 碰撞平滑  │  │ FOV 可见体素→2D 占用图  │
           ├─────────────────────┤  ├──────────────────────┤
  3D A*    │ 模式3: 全图 26连通   │  │ 模式4: FOV 内 3D 搜索  │
           │ 3D 体素 A*          │  │ 不可见体素 = 占用       │
           └─────────────────────┘  └──────────────────────┘
```

### 切换方式

**地面站**（推荐）：右侧 Perception & Planning 面板 → Mode 下拉选 Global/FOV、Dim 下拉选 2D/3D

**命令行**：
```bash
ros2 param set /drone_planner perception_mode fov     # 切换 FOV 模式
ros2 param set /drone_planner planner_dim 3d          # 切换 3D A*
ros2 param set /drone_planner fov_h_deg 90.0          # 修改水平 FOV
ros2 param set /drone_planner fov_range 8.0           # 修改感知距离
```

### 模式详情

| 模式 | 适用场景 | 特点 |
|---|---|---|
| 1: Global+2D | 默认模式，全图避障 | 8 连通 A* + octile 启发 + Bresenham 碰撞感知平滑 |
| 2: FOV+2D | 模拟机载相机局部感知 | FOV 锥体内可见体素投影到 2D 栅格，不可见区域视为未知（不阻塞） |
| 3: Global+3D | 全图 3D 避障 | 26 连通 3D A* + 3D octile 启发 + 3D Bresenham 平滑 |
| 4: FOV+3D | 最严格局部感知 | FOV 内进行 3D A*，不可见体素标记为占用（保守安全） |

### safe_goal 单调推进

规划器每 replan 周期（默认 1 Hz）重规划整条路径，但 safe_goal **最多前进 1 个航点**。这防止了路径跳变导致的控制器震荡。到达终点时 planner_status 变为 `AT_GOAL`。

### 规划器参数

在 `drone_planner/config/planner.yaml`：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `resolution` | 0.1 | 2D 栅格分辨率 m |
| `bounds` | [-3,5,-3,5] | 地图区域 [x0,x1,y0,y1] |
| `safety_distance` | 0.4 | 障碍物安全距离 m |
| `drone_radius` | 0.2 | 无人机外接圆 m |
| `replan_rate` | 1.0 | 重规划频率 Hz |
| `advance_tol` | 0.15 | 到达航点的距离阈值 m |
| `perception_mode` | global | "global" / "fov" |
| `planner_dim` | 2d | "2d" / "3d" |
| `fov_h_deg` | 60.0 | 水平视场角 (°) |
| `fov_v_deg` | 45.0 | 垂直视场角 (°) |
| `fov_range` | 5.0 | 感知距离 m |
| `voxel_resolution` | 0.2 | 3D 体素分辨率 m |
| `z_bounds` | [0.0, 4.0] | 体素 z 范围 |

---

## 地图与障碍物

### 默认 6 个障碍物（显式 YAML 配置）

`drone_map/config/map.yaml`:
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

### 狭窄通道场景

`map.yaml` 底部注释中包含窄通道配置——两排 r=0.15 柱体墙形成 0.8m 宽通道。取消注释并注释默认障碍物即可切换。

### 程序随机生成（固定种子，可复现）

编辑 `map.yaml`，把 `procedural: false` 改成 `procedural: true`，然后改这些参数：

| 参数 | 默认 | 含义 |
|---|---|---|
| `seed` | 42 | 随机种子（相同 seed = 相同地图） |
| `num_obstacles` | 12 | 障碍物目标数量 |
| `num_spheres` | 3 | 球数量（-1=不限制） |
| `num_cylinders` | 3 | 柱数量 |
| `num_cubes` | 2 | 立方数量 |
| `bounds` | [-2, 4, -2, 4, 0, 5] | 生成区域 [x0 x1 y0 y1 z0 z1] |
| `clear_radius` | 0.8 | 障碍物离起点和终点的最小距离 |
| `size_min` / `size_max` | 0.2 / 0.5 | 障碍物尺寸范围 |

### 运行时动态切换（不用重启）

```bash
ros2 param set /drone_map procedural true
ros2 param set /drone_map seed 77
ros2 param set /drone_map num_obstacles 10
# 障碍物即时更新，RViz 里立即看到变化
```

---

## 风扰与传感器噪声

通过地面站面板或 `ros2 param set` 动态启用，无需重启。

### 风扰

模拟恒定风 + 正弦阵风 + Ornstein-Uhlenbeck 随机湍流：

```bash
ros2 param set /drone_dynamics wind_enabled true
ros2 param set /drone_dynamics wind_force "[2.0,0.0,0.0]"
ros2 param set /drone_dynamics wind_gust_amplitude 1.0
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `wind_enabled` | false | 启用风扰 |
| `wind_force` | [0,0,0] | 恒定风力 (N, 世界系) |
| `wind_gust_amplitude` | 0.0 | 阵风幅值 (N) |
| `wind_gust_period` | 2.0 | 阵风周期 (s) |

控制器通过积分项 (`Ki_pos.x=0.5`) 主动补偿恒定风扰。

### 传感器噪声

IMU 加速度计/陀螺仪偏置 + 白噪声 + 随机游走，以及 Odom 位置/速度加性噪声：

```bash
ros2 param set /drone_dynamics imu_noise_enabled true
ros2 param set /drone_dynamics accel_noise_density 0.05
ros2 param set /drone_dynamics gyro_noise_density 0.01
ros2 param set /drone_dynamics accel_bias_init 0.1
ros2 param set /drone_dynamics odom_pos_noise 0.02
```

| 参数 | 默认 | 含义 |
|---|---|---|
| `imu_noise_enabled` | false | 启用 IMU 噪声 |
| `accel_noise_density` | 0.0 | 加速度计白噪声 σ (m/s²) |
| `gyro_noise_density` | 0.0 | 陀螺仪白噪声 σ (rad/s) |
| `accel_bias_init` / `gyro_bias_init` | 0 | 初始偏置 |
| `accel_bias_rw` / `gyro_bias_rw` | 0 | 偏置随机游走 σ |
| `odom_pos_noise` / `odom_vel_noise` | 0 | Odom 位置/速度噪声 σ |

**注意**：TF 始终使用 ground truth，不受噪声影响。

---

## 实时监控仪表盘（地面站）

浏览器仪表盘，实时显示位置误差、RPM、轨迹、障碍物距离 + **交互式控制面板**，无需额外依赖。

### 启动

```bash
# 终端 1：仿真栈
ros2 launch drone_bringup full.launch.py

# 终端 2：监控器（⚠️ 必须用系统 Python，source ROS 环境）
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
| 3. XY Trajectory | 无人机轨迹（绿线）+ 障碍物（彩色圆）+ Safe Goal（黄点） | 轨迹不穿透障碍物 |
| 4. Obstacle Distances | 各障碍物距离（彩色线）+ 最近距离（绿粗线）+ 安全线 0.4m（红虚线） | 最近距离 > 0.4m |

### 控制面板

**🎯 目标点下发**：输入 x/y/z 点击 Send Goal，或点预设按钮（Home / TargetA / Square / Up）

**🔍 感知与规划**：
- Mode 下拉：Global ↔ FOV 实时切换
- Dim 下拉：2D A* ↔ 3D A* 实时切换
- FOV H/V/Range 滑块：调整视场角和感知距离，拖拽即生效

**⚙ 控制器热调参**：8 个参数滑块（Kp_pos、Kd_pos、Kp_att、Kd_rate、a_xy_max、a_z_max），拖动后点「Apply All」通过 `ros2 param set` 即时生效

**🌬 风扰面板**：启用/禁用 + 恒定力 + 阵风强度，预设 Light/Medium/Strong

**📡 传感器噪声面板**：IMU/Odom 噪声开关 + 各通道 σ，预设 Light/Realistic

**🛑 紧急操作**：
- STOP MOTORS — 向 `/drone/motor_rpm_cmd` 发送零 RPM
- Return to Origin — 下发目标点 (0,0,1.5)

### 退出

`Ctrl+C` 退出，自动保存 `dashboard_log.csv`（含全部时间序列数据，可用 Excel/Python 二次分析）。

---

## 录制实验数据

### 快速录制

每个验收场景一条命令，自动录 bag + 打印数值摘要：

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

### rosbag 录制

```bash
ros2 bag record -o bag_experiment \
  /drone/odom /drone/imu /drone/motor_rpm_cmd /drone/goal \
  /drone/safe_goal /drone/path /drone/planned_path \
  /map/obstacles /drone/planner_status /tf
```

### 数据分析

```bash
# bag → CSV + 数值摘要（稳态误差、超调量、settling time、RPM 统计）
python3 src/astesia_drone_sim/drone_bringup/scripts/plot_bag.py bag_experiment

# 仅打印数值摘要
python3 src/astesia_drone_sim/drone_bringup/scripts/plot_bag.py bag_experiment --summary
```

---

## 自动验收脚本

`auto_eval.py` 自动运行 6 个验收场景并产出 `eval_report.json`：

```bash
ros2 run drone_bringup auto_eval.py
```

| # | 场景 | 查验标准 | Launch |
|---|---|---|---|
| 1 | 悬停 10s | 稳态 z 误差 < 0.3m | no_planner |
| 2 | 飞向 (2,1,1.5) | 终点误差 < 0.3m | no_planner |
| 3 | 正方形航线 4 点 | ≥3 点到达误差 < 0.3m | no_planner |
| 4 | 静态避障 | min_obs_dist > 0.4m | full |
| 5 | 狭窄通道绕行 | 最终位置误差 < 1.0m | full |
| 6 | 稳定性 | 位置误差标准差 < 0.1m | no_planner |

---

## 多无人机

支持命名空间隔离的多机场景：

```bash
ros2 launch drone_bringup multi_drone.launch.py
```

默认启动 2 架：`drone0` 在原点、`drone1` 在 (1.5, 0, 0)。每架有独立的 dynamics + controller，共享同一份地图。通过命名空间话题控制：

```bash
ros2 topic pub --once /drone0/drone/goal geometry_msgs/msg/PoseStamped ...
```

---

## 运行单元测试

```bash
colcon test --packages-select drone_common --event-handlers console_direct+
```

应看到 `[PASSED] 7 tests`——验证 mixer 矩阵 (B·B⁻¹≈I)、四元数积分、roll/yaw/pitch 符号正确。

---

## 目录结构

```
src/astesia_drone_sim/
├── drone_common/               # 共享头文件库（header-only，不依赖 ROS）
│   ├── include/drone_common/
│   │   ├── frames.hpp          #   坐标系 + 电机布局（所有节点的单信源）
│   │   ├── mixer.hpp           #   控制分配矩阵 B / B⁻¹
│   │   ├── math.hpp            #   四元数积分 / skew / vee / rotationFromBodyZAndYaw
│   │   ├── obstacle.hpp        #   障碍物结构 (Sphere/Cylinder/Cube) + 碰撞检测
│   │   ├── noise.hpp           #   IMU/Odom 传感器噪声模型
│   │   ├── voxel.hpp           #   3D 体素栅格 + FOV 锥体查询
│   │   └── drone_common.hpp    #   聚合 include
│   ├── test/test_common.cpp    #   7 条 gtest 回归网
│   └── CMakeLists.txt          #   INTERFACE library + ament_export_targets
├── drone_dynamics/             # 动力学节点（1kHz 积分 + 100Hz 发布）
│   ├── include/drone_dynamics/dynamics.hpp
│   ├── src/dynamics_node.cpp   #   MT executor(2) + 独立 callback group
│   └── config/dynamics.yaml
├── drone_controller/           # 控制器节点（200Hz）
│   ├── include/drone_controller/
│   │   ├── controller.hpp      #   线性 PD + 死区积分 + 反饱和 + Lee 几何姿态
│   │   └── ladrc.hpp           #   二阶 LADRC（实验性 bonus）
│   ├── src/controller_node.cpp
│   └── config/controller.yaml
├── drone_map/                  # 地图节点（1Hz 发布 + 热切换）
│   ├── src/map_node.cpp        #   显式 YAML / 程序化随机障碍物
│   └── config/map.yaml
├── drone_planner/              # 规划器节点（1Hz 重规划，四模式）
│   ├── include/drone_planner/
│   │   ├── grid.hpp            #   2D 膨胀栅格构建
│   │   ├── astar.hpp           #   8 连通 A* + Bresenham 碰撞感知平滑
│   │   ├── astar3d.hpp         #   26 连通 3D A* + 3D octile 启发
│   │   └── perception.hpp      #   PerceptionEngine — 四模式路由
│   ├── src/planner_node.cpp    #   + FOV 锥体/体素可视化
│   └── config/planner.yaml
├── drone_bringup/              # 启动与工具
│   ├── launch/
│   │   ├── full.launch.py      #   完整系统（+ 规划器）
│   │   ├── no_planner.launch.py #  基础系统（控制器直连 goal）
│   │   └── multi_drone.launch.py  # 多无人机（命名空间隔离）
│   ├── config/
│   │   └── parameter_reference.yaml # 统一参数参考（全参数一览）
│   ├── rviz/drone.rviz         #   RViz 预设配置
│   └── scripts/
│       ├── live_monitor.py     #   地面站（HTTP + rclpy + Chart.js）
│       ├── waypoint_sender.py  #   多航点顺序发送
│       ├── trajectory_generator.py # 参数化轨迹生成器
│       ├── record_and_analyze.sh # 录 bag + 自动分析
│       ├── plot_bag.py        #   bag → CSV + 数值摘要
│       ├── auto_eval.py       #   6 场景自动验收
│       └── ladrc_autotune.py  #   LADRC 自适应整定（实验性）
├── report/
│   ├── report.md              #   技术报告
│   ├── verification_guide.html #  验收场景录制指南
│   └── README.md
├── ai_usage.md                 # AI 辅助编程说明
├── README.md                   # 本文件
└── .gitignore
```

---

## 常见故障排查

### 1. `colcon build` 报错 `No module named 'catkin_pkg'`

**原因**：Anaconda/Miniconda 的 Python 3.11 劫持了 cmake。

**修复**：**每次** colcon build 必须带完整参数：
```bash
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
```
建议写 alias：
```bash
alias cb='colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3'
```

### 2. `ros2 param set` 报 `rclpy.ok()` 错误

**原因**：同上——conda 的 Python 破坏了 ROS2 CLI。

**修复**：运行 `ros2` 命令前先 `conda deactivate`（或开新终端，不 activate conda）。

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
2. 如果只有 1 个：检查 `drone_map/config/map.yaml` 的 `procedural` 和 `obstacles` 列表
3. `ros2 param set /drone_map procedural true` 切到随机生成看是否生效

### 6. 僵尸进程（之前的节点没杀干净）

```bash
# 一键清掉本项目所有节点
pkill -9 dynamics_node controller_node map_node planner_node rviz2 ros2
```

### 7. 地面站无法绑定端口 8765

```bash
fuser -k 8765/tcp
```

---

## 更多信息

- **技术报告**: `report/report.md`
- **验收场景录制指南**: 浏览器打开 `report/verification_guide.html`
- **AI 使用说明**: `ai_usage.md`
- **统一参数参考**: `drone_bringup/config/parameter_reference.yaml`
- **GitHub**: https://github.com/bestastesia/astesia_drone_sim

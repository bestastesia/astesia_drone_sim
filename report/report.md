# asteesia_drone_sim — 四旋翼无人机仿真器技术报告

> 2027 保研招生编程题（60 分）

## 1. 系统架构

### 节点拓扑

```
                      ┌──────────────┐
                      │   RViz2      │  ← 可视化
                      └──────┬───────┘
                             │ odom / imu / path / safe_goal / obstacles / tf
                             │ planned_path / fov_cone / fov_voxels
        ┌────────┐    ┌──────┴───────┐    ┌──────────┐
        │ drone_ │    │   drone_      │    │  drone_  │
        │  map   │───→│  planner     │───→│controller│
        │ Marker │    │ 4-mode A*   │    │ PD+mixer │
        │ Array  │    └──────┬───────┘    └────┬─────┘
        └────────┘           │ safe_goal       │ rpm_cmd
                             │                 │
                        ┌────┴─────────────────┴────┐
                        │      drone_dynamics        │
                        │  EOM + motor 1st-order     │
                        │  + wind + IMU noise        │
                        └───────────┬────────────────┘
                                    │ odom / imu / path / tf
                                    │
                              ┌─────┴──────┐
                              │   RViz2     │
                              └────────────┘
```

### 话题表

| Topic | 类型 | 方向 |
|---|---|---|
| /drone/motor_rpm_cmd | Float32MultiArray [FL,FR,BL,BR] | controller → dynamics |
| /drone/odom | nav_msgs/Odometry | dynamics → controller, planner, rviz |
| /drone/imu | sensor_msgs/Imu | dynamics → rviz |
| /drone/path | nav_msgs/Path | dynamics → rviz |
| /drone/goal | geometry_msgs/PoseStamped | user/rviz → planner |
| /drone/safe_goal | geometry_msgs/PoseStamped | planner → controller |
| /map/obstacles | visualization_msgs/MarkerArray | map → planner, rviz |
| /drone/planned_path | nav_msgs/Path | planner → rviz |
| /drone/planner_status | std_msgs/String | planner → rviz, ground station |
| /drone/fov_cone | visualization_msgs/Marker (LINE_LIST) | planner → rviz |
| /drone/fov_voxels | visualization_msgs/MarkerArray (CUBE_LIST) | planner → rviz |

### tf 树
```
map (fixed) → base_link (dynamics broadcaster, 100Hz)
            → imu_link (static, identity)
```

## 2. 动力学模型

X 型四旋翼，REP-103 ENU 世界系：

- **状态量**: p(xyz), v(xyz), q_WB(wxyz), ω_B(xyz), motor_ω[4]
- **推力**: F_i = k_F·ω_i²（沿机体+z）
- **力矩**: τ = Σ r_i × F_i + k_M·sgn_i·ω_i²（推力臂贡献 roll/pitch + 反扭矩 yaw）
- **平动**: a_W = R_WB·(F/m)·ẑ_B − g·ẑ_W
- **转动**: ω̇_B = I⁻¹(τ − ω×(I·ω))
- **电机一阶**: ω̇_i = (ω_cmd_i − ω_i)/τ_m, clamp [ω_min, ω_max]
- **积分**: 半隐式 Euler, dt=1ms, 每步四元数归一化

Mixer 矩阵 B (行[F,τx,τy,τz], 列[ω₁²..ω₄²]):
```
B = [  kF      kF      kF      kF    ]
    [ +a     -a      +a     -a     ]   a = l/√2·kF
    [ -a     -a      +a     +a     ]
    [ +kM    -kM     -kM     +kM    ]
```
B⁻¹ 预计算缓存，dynamics 与 controller 共享同一份头文件。

## 3. 控制器设计

### 位置环（线性 PD + 有限积分 + 反饱和）

- `e_p = p_goal − p`, `e_v = −v` (hover/waypoint-hold: v_des=0)
- **PD 项**: `a_des = Kp·e_p + Kd·e_v`
- **积分项**: 内置 0.02m 死区 — 小误差时缓慢衰减积分(×0.99/step)；遇限削弱 conditional anti-windup — 仅当 PD+积分不超限幅时累加；积分限幅 Ki_max=2.0 m/s²
- **风扰补偿**: x 轴 Ki=0.5，稳态抵消恒力风扰
- 参数: Kp=[2.0,2.0,3.0], Kd=[2.0,2.0,2.4] (ζ≈0.7 阻尼比)
- Z 轴 Kp=3.0, Kd=2.4（上升优先级高于水平）

### 姿态环（几何 Lee 误差，避免 Euler 奇异）
- `T_W = m(a_des + g)` → `z_B_des = normalize(T_W)`
- `R_des` 由 `z_B_des + yaw` 构造（世界系投影法，避免 gimbal lock）
- `e_R = ½ vee(R_desᵀR − RᵀR_des)`, `τ = −Kp_R·e_R − Kd_ω·e_ω`
- Kp_att=[8,8,3], Kd_rate=[0.8,0.8,0.7]

### Mixer 逆
- `u = B⁻¹·[F, τx, τy, τz]ᵀ` → `ω²_i = clamp(u_i, 0, ω_max²)` → `RPM = ω·60/2π`

### 限幅链
a_des 每轴 clamp → far-goal failsafe → F clamp → τ clamp → mixer → ω² clamp → RPM clamp

### LADRC（实验性 bonus）
二阶线性自抗扰控制，每轴独立 LESO 估计总扰动并实时补偿。`control_mode: "ladrc"` 可切换，默认 `"pd"`（推荐）。

## 4. 感知与路径规划（四模式）

### 障碍物表示
- Sphere / Cylinder / Cube 三种几何体，轴对齐
- 膨胀半径 = safety_distance(0.4m) + drone_radius(0.2m) = 0.6m
- 6 个显式障碍物置于起点(0,0,1.5)→目标(2,1,1.5)直线上及其两侧
- 支持程序化随机生成（固定 seed 可复现）和 YAML 显式列表

### 四模式矩阵

```
              全局感知 (GLOBAL)           局部感知 (FOV)
           ┌─────────────────────┐  ┌──────────────────────┐
  2D A*    │ 模式1: 全局 2D       │  │ 模式2: FOV→2D 栅格投影 │
           │ 8连通 A* + Bresenham │  │ FOV 锥体内体素→2D 占用  │
           ├─────────────────────┤  ├──────────────────────┤
  3D A*    │ 模式3: 全图 26连通   │  │ 模式4: FOV 内 3D 搜索  │
           │ 3D octile A*        │  │ 不可见体素 = 占用       │
           └─────────────────────┘  └──────────────────────┘
```

### 模式 1: 全局 2D A*（默认）
- 2D 膨胀栅格 (resolution 0.1m)，z_cruise = goal.z
- z 跨度检查：障碍物高度不含巡航高度的忽略（3D 意识）
- 8 连通 A*，octile heuristic
- **碰撞感知平滑**：Bresenham 线检查，只跳掉安全直线段；平滑坍塌时保留原始路径

### 模式 2: FOV + 2D
- 无人机前方 FOV 锥体（水平/垂直视场角可配）查询体素可见性
- 可见的占用体素投影到 2D 栅格
- 不可见区域视为未知（不阻塞），模拟机载相机局部感知

### 模式 3: 全局 3D A*
- 26 连通 3D A*，3D octile 启发
- 3D Bresenham 碰撞感知平滑
- BFS 最近自由格（半径 20）处理起点/终点在占用栅格中的情况

### 模式 4: FOV + 3D
- FOV 锥体内进行 26 连通 3D A*
- 不可见体素标记为占用（保守安全策略）
- 最严格的局部感知模式

### safe_goal 单调推进
- safe_goal = 路径上紧接最近航点的下一个点（最多前进 1 个航点/周期）
- 防止路径跳变导致的控制器震荡
- 到达终点时 planner_status = AT_GOAL

### 失败条件
- NO_PATH: 无可行路径 → 保留 last safe_goal, controller 悬停
- NO_ODOM: 无里程计 → 不规划
- WAITING_FOR_MAP: 无地图 → 发布原 goal 为 safe_goal

### 感知参数（ros2 param set 热切换）
- fov_h_deg: 水平视场角 (°, 默认 60)
- fov_v_deg: 垂直视场角 (°, 默认 45)
- fov_range: 感知距离 (m, 默认 5.0)
- voxel_resolution: 体素分辨率 (m, 默认 0.2)
- z_bounds: 体素 z 范围 (默认 [0.0, 4.0])

### FOV 可视化（RViz）
- `/drone/fov_cone`: LINE_LIST 青色线框锥体
- `/drone/fov_voxels`: CUBE_LIST 绿色可见占用体素

## 5. 传感器噪声与风扰

### 风扰模型
- 恒定风力 + 正弦阵风 + Ornstein-Uhlenbeck 随机湍流
- 所有参数支持 `ros2 param set` 热更新
- 控制器通过 x 轴积分项 (Ki=0.5) 主动补偿恒定风扰

### IMU 噪声
- 加速度计/陀螺仪偏置 + 高斯白噪声 + 偏置随机游走
- 可通过地面站面板或 ros2 param set 实时启用/调整

### Odom 噪声
- 位置/速度独立加性高斯噪声
- TF 始终使用 ground truth（不受噪声影响）

## 6. 可视化方案 (RViz2 + 地面站)

### RViz2
- 无人机: ARROW Marker (绿色，指向机头)
- 障碍物: MarkerArray (橙色半透明球/柱/立方)
- 轨迹: /drone/path (绿色历史) + /drone/planned_path (黄色规划)
- 目标点: /drone/goal (红色) + /drone/safe_goal (蓝色)
- FOV: /drone/fov_cone (青色线框锥体) + /drone/fov_voxels (绿色体素)
- 工具栏: 2D Goal Pose 插件 → /drone/goal

### 地面站 (live_monitor.py)
- 浏览器仪表盘 (localhost:8765)，零额外依赖
- 4 个实时图表: 位置误差 / RPM / XY 轨迹 / 障碍物距离
- 交互式控制面板: 目标点下发、感知模式切换、FOV 参数滑块、控制器热调参、风扰/噪声面板、紧急停机
- Ctrl+C 退出时自动保存 dashboard_log.csv

## 7. 实验结果

### 验收场景

| # | 场景 | 结果 |
|---|---|---|
| 1 | 悬停 (0,0,1.5) | 稳态误差 <0.02m ✓ |
| 2 | 目标点 (2,1,1.5) | 收敛到 <0.01m ✓ |
| 3 | 正方形航线 (4 点) | 每个点停留收敛 ✓ |
| 4 | 静态避障 (6 障碍物) | A* 路径绕行，安全距离 >0.6m ✓ |
| 5 | 狭窄通道绕行 | A* 找到绕行路径 ✓ |
| 6 | 稳定性曲线 | 见 CSV 数据（地面站自动保存） |

### 关键指标

| 指标 | 数值 |
|---|---|
| 最终位置误差 | <0.02m (3σ) |
| 最大超调量 (z) | <0.2m |
| 稳态误差 | <0.01m |
| 最小障碍物距离 | >0.6m (膨胀后) |
| RPM 稳态 | 14.9-15.1 (hover) |

### 感知模式

| 模式 | 说明 |
|---|---|
| Global+2D | 默认，全局 8 连通 A*，成熟稳定 |
| FOV+2D | FOV 锥体→2D 投影，模拟机载相机 |
| Global+3D | 26 连通 3D A*，全图立体避障 |
| FOV+3D | FOV 内 3D A*，最严格局部感知 |

## 8. 与参考仓库的关系

### pengyu_sim 参考点
- 动力学公式参考（EOM、mixer）
- 未使用其 ROS1 代码，全部重写为 ROS2 C++

### MARSIM 参考点
- 点云/障碍物思路参考
- 未使用其代码，改用更简单的显式几何障碍物

### 为什么这样设计
- 全 C++ rclcpp：避免 Python 版本冲突（conda 3.11 vs ROS2 3.10）
- header-only drone_common：mixer 单信源，杜绝符号错翻车
- 2D/3D 膨胀栅格 + 四模式感知：从简单全局 2D 到严格 FOV+3D，逐级提升复杂度
- MT executor + 隔离 callback group：1kHz dynamics 与订阅不互锁
- 地面站内嵌 HTTP 服务器：零依赖（无 pip/flask），浏览器即开即用

## 9. AI 使用说明

详见 ai_usage.md。

## 10. 反思：如果继续做两周

1. **轨迹跟踪**：当前 waypoint-by-waypoint 方法在航点切换时有惯性弯曲。可改用 minimum-snap trajectory + 模型预测控制
2. **LADRC 完善**：当前独立三轴 LADRC 忽略了姿态内环耦合。可改用 cascaded LADRC
3. **多无人机协同**：当前仅命名空间隔离，可加 inter-drone collision avoidance
4. **单元测试扩展**：当前仅 drone_common 有 gtest。dynamics/controller/planner 应加同样的回归测试
5. **CI/CD**：GitHub Actions 自动 colcon build + test，每次 push 触发
6. **物理引擎**：可对接 Gazebo/MuJoCo 替代手写 EOM，实现更真实的传感器仿真
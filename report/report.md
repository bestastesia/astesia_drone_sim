# asteesia_drone_sim — 四旋翼无人机仿真器技术报告

> 2027 保研招生编程题（60 分）

## 1. 系统架构

### 节点拓扑

```
                  ┌──────────────┐
                  │  RViz2       │  ← 可视化
                  └──────┬───────┘
                         │ odom/path/goal/safe_goal/obstacles/tf
    ┌────────┐    ┌──────┴───────┐    ┌──────────┐
    │ drone_ │    │   drone_      │    │  drone_  │
    │  map   │───→│  planner     │───→│controller│
    │ Marker │    │ A* infl grid │    │ casc PD  │
    │ Array  │    └──────┬───────┘    └────┬─────┘
    └────────┘           │ safe_goal       │ rpm_cmd
                         │                 │
                    ┌────┴─────────────────┴────┐
                    │        drone_dynamics      │
                    │   EOM + motor 1st-order    │
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
| /drone/planner_status | std_msgs/String | planner → rviz |

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

### 位置环（PD）
- `e_p = p_goal − p`  →  `a_des = Kp·e_p + Kd·(−v)`
- 参数: Kp=2.5, Kd=2.2 (ζ≈0.7)；Z 轴 Kp=4.0, Kd=2.8

### 姿态环（几何 Lee 误差，避免 Euler 奇异）
- `T_W = m(a_des + g)` → `z_B_des = normalize(T_W)`
- `R_des` 由 `z_B_des + yaw` 构造
- `e_R = ½ vee(R_desᵀR − RᵀR_des)`, `τ = −Kp_R·e_R − Kd_ω·e_ω`

### Mixer 逆
- `u = B⁻¹·[F, τx, τy, τz]ᵀ` → `ω²_i = clamp(u_i, 0, ω_max²)` → `RPM = ω·60/2π`

### 限幅链
a_des 每轴 clamp → far-goal failsafe → F clamp → τ clamp → mixer → ω² clamp → RPM clamp

## 4. 地图与避障

### 障碍物表示
- 6 个静态障碍物 (sphere/cylinder/cube)，YAML 显式列表
- 膨胀半径 = safety_distance(0.4m) + drone_radius(0.2m) = 0.6m
- 置于起点(0,0,1.5)→目标(2,1,1.5)直线上及其两侧

### 路径规划 A*
- 2D 膨胀栅格 (resolution 0.1m)，z_cruise = goal.z
- z 跨度检查：障碍物高度不含巡航高度的忽略（3D 意识）
- 8 连通 A*，octile heuristic
- **碰撞感知平滑**：Bresenham 线检查，只跳掉安全直线段
- safe_goal = 路径上紧接最近航点的下一个点（+1，不跳点）

### 失败条件
- GOAL_IN_OBSTACLE: 目标格被占 → 发原目标，controller 悬停
- NO_PATH: 膨胀后会阻塞 → 保留 last safe_goal

## 5. 可视化方案 (RViz2)

- 无人机: ARROW Marker (绿色，指向机头)
- 障碍物: MarkerArray (橙色半透明球/柱/立方)
- 轨迹: /drone/path (绿色) + /drone/planned_path (黄色)
- 目标点: /drone/goal + /drone/safe_goal
- 工具栏: 2D Goal Pose 插件 → /drone/goal

## 6. 实验结果

### 验收场景

| # | 场景 | 结果 |
|---|---|---|
| 1 | 悬停 (0,0,1.5) | 稳态误差 <0.02m ✓ |
| 2 | 目标点 (2,1,1.5) | 收敛到 0.004m ✓ |
| 3 | 正方形航线 (4 点) | 每个点停留收敛 ✓ |
| 4 | 静态避障 (6 障碍物) | A* 路径绕行，GOAL free ✓ |
| 5 | 狭窄通道绕行 | A* 找到 7 航点绕行路径 ✓ |
| 6 | 稳定性曲线 | 见附录 CSV 数据 |

### 关键指标

| 指标 | 数值 |
|---|---|
| 最终位置误差 | <0.02m (3σ) |
| 最大超调量 (z) | <0.2m |
| 稳态误差 | <0.01m |
| 最小障碍物距离 | >0.6m (膨胀后) |
| RPM 稳态 | 14.9-15.1 (hover) |

## 7. 与参考仓库的关系

### pengyu_sim 参考点
- 动力学公式参考（EOM、mixer）
- 未使用其 ROS1 代码，全部重写为 ROS2 C++

### MARSIM 参考点
- 点云/障碍物思路参考
- 未使用其代码，改用更简单的显式几何障碍物

### 为什么这样设计
- 全 C++ rclcpp：避免 Python 版本冲突（conda 3.11 vs ROS2 3.10）
- header-only drone_common：mixer 单信源，杜绝符号错翻车
- 2D 膨胀栅格 A*：比 3D 简单，在碰撞感知平滑后等效安全
- MT executor + 隔离 callback group：1kHz dynamics 与订阅不互锁

## 8. AI 使用说明

详见 ai_usage.md。

## 9. 反思：如果继续做两周

1. **LADRC 完善**：当前独立三轴 LADRC 忽略了姿态内环耦合。两周内可改用 cascaded LADRC（外环 LADRC 输出姿态角、内环 LADRC 跟踪姿态），并加入控制量饱和反馈到 LESO
2. **轨迹跟踪**：当前 waypoint-by-waypoint 方法在航点切换时有惯性弯曲。可改用 minimum-snap trajectory + 模型预测控制
3. **传感器噪声**：仿真中 odom/imu 直接出真值，加入传感器噪声模型能让控制器验证更真实
4. **多无人机**：当前架构单机，drone_common 的命名空间设计已有预留
5. **单元测试扩展**：当前仅 drone_common 有 gtest。dynamics/controller 应加同样的回归测试
6. **CI/CD**：GitHub Actions 自动 colcon build + test，每次 push 触发
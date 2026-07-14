# asteesia_drone_sim

ROS2 Humble 四旋翼无人机仿真器（2027 保研招生编程题 · 60 分）。

## 一键启动

```bash
cd /home/astesia/drone_sim_ws
colcon build --symlink-install --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash

# 完整系统（动力学 + 控制器 + 地图 + 规划避障 + RViz）
ros2 launch drone_bringup full.launch.py

# 基础系统（无规划器，PD 直飞）
ros2 launch drone_bringup no_planner.launch.py
```

> ⚠️ **conda 用户注意**：每次 `colcon build` 必须带 `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`，否则 conda 的 Python 3.11 会劫持 cmake。

## 发送目标点

**RViz**：工具栏 → 2D Goal Pose → 在 map 上点位置 → z 改成 1.5 → 确定

**命令行**：
```bash
ros2 topic pub --once /drone/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: "map"}, pose: {position: {x: 2.0, y: 1.0, z: 1.5}}}'
```

**多航点正方形航线**：
```bash
ros2 run drone_bringup waypoint_sender.py --ros-args -p hold_time:=4.0
```

## 包结构

```
src/astesia_drone_sim/
├── drone_common/       # header-only: mixer / frames / math / obstacle
├── drone_dynamics/     # 动力学积分 (EOM + 电机一阶) + odom/imu/tf 发布
├── drone_controller/   # 级联 PD + 几何姿态控制 + mixer 逆
├── drone_map/          # 静态障碍物 (6x sphere/cylinder/cube) + MarkerArray
├── drone_planner/      # 2D 膨胀栅格 A* + 碰撞感知平滑 + safe_goal
└── drone_bringup/      # launch / config / rviz / scripts / report
```

## 验收场景

| # | 场景 | 要求 | 结果 |
|---|---|---|---|
| 1 | 悬停 (0,0,1.5) | 稳态 < 0.3 m | ✅ 0.0001 m |
| 2 | 目标点 (2,1,1.5) | 收敛 | ✅ 0.004 m |
| 3 | 正方形航线 | 4 点顺序流转 | ✅ |
| 4 | 静态避障 | 6 障碍物，路径绕行 | ✅ |
| 5 | 狭窄通道 | 路径明显弯曲 | ✅ |
| 6 | 稳定性曲线 | 误差/RPM/轨迹/距离 | 见 report/ |

## 录制数据 + 分析

```bash
# 每个场景一条命令，自动录 bag + 打印数值摘要
bash src/astesia_drone_sim/drone_bringup/scripts/record_and_analyze.sh bag_hover 8
```

详见 `report/verification_guide.html` 或 `report/report.md`。

## 运行单元测试

```bash
colcon test --packages-select drone_common  # mixer/frames/math 回归网
```

## 环境

- Ubuntu 22.04 + ROS2 Humble
- C++17 + Eigen 3.4 + rclcpp
- `colcon build --symlink-install`
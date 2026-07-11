# asteesia_drone_sim

ROS2 Humble 四旋翼无人机仿真器（2027 保研招生编程题）。

## 状态
- [ ] Step 0 工作区骨架
- [ ] Step 1 drone_common
- [ ] Step 2 drone_dynamics
- [ ] Step 3 drone_controller（悬停）
- [ ] Step 4 go-to-goal / 正方形航线
- [ ] Step 5 drone_map
- [ ] Step 6 drone_planner
- [ ] Step 7 plot_bag / report / ai_usage

## 一键启动（占位，逐步补全）
```bash
cd /home/astesia/drone_sim_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch drone_bringup full.launch.py
```

## 包结构
```
src/astesia_drone_sim/
├── drone_common/       # header-only: mixer / frames / math / obstacle
├── drone_dynamics/     # 动力学积分 + sim tick
├── drone_controller/   # 级联 PD 控制 + mixer 逆
├── drone_map/          # 静态障碍物 + MarkerArray
├── drone_planner/      # 2D 膨胀栅格 A*
└── drone_bringup/      # launch / config / rviz / scripts / report
```

## 验收场景
1. 悬停 (0,0,1.5)，稳态误差 < 0.3 m
2. 目标点 (2,1,1.5)
3. 多目标点正方形航线
4. 静态避障（≥5 障碍物）
5. 狭窄通道绕行
6. 稳定性曲线（位置误差 / RPM / 轨迹 / 最小障碍物距离）

详见 `report/`。
# AI 辅助编程说明（ai_usage.md）

任务要求如实说明 AI 使用情况。本文件随实现进度滚动更新。

## 1. 使用的 AI 工具
- Claude Code（Anthropic，claude-opus 模型）——主交互工具，用于架构设计、代码生成、调试、单元测试编写与文档。

## 2. 关键 prompt / 交互摘要（累计，≥8 条）
1. 「阅读 `/home/astesia/文档/output.pdf` 并 plan」——让 AI 提取任务要求并产出实现计划。
2. 「你的全权限通过指令是什么」「`claude-nexus` 意味着什么」等此前会话——确认运行环境与工具链认知。
3. 「Plan agent 设计 ROS2 Humble 四旋翼仿真器架构」——产出 6 包布局、mixer 公式、推导、launch/bag/plot 流程、数值陷阱清单。
4. 「仓库名 astesia_drone_sim；每完成模块端到端验证+人在环+git 推送」——确立增量交付节奏。
5. 指导 AI 修复 colcon 在 conda Python 下报 `No module named 'catkin_pkg'`——剥 PATH + `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`。
6. 指导 AI 修复 header-only 包 downstream 找不到 `drone_common::drone_common` target——改为 INTERFACE library + `ament_export_targets`。
7. 「在 /tmp 写 mixer 自检，先不依赖 ROS 用 g++ 跑」——产出 B·B⁻¹≈I 与 roll/yaw/pitch 符号断言。
8. 「把自检落成 ament_cmake_gtest 进 `colcon test`」——7 条 gtest 全过，作为后续 mixer 改动的回归网。
9. 指导 git：`ssh-keygen ed25519` → 加 GitHub SSH key → `remote set-url` → `git push -u origin main`。

## 3. AI 帮我完成的模块
- 工作区与 6 个 ament_cmake 包骨架（package.xml/CMakeLists 脚手架）。
- drone_common 全部头文件（frames/mixer/math/obstacle）与其单元测试。
- git 仓库初始化与首次推送。

## 4. 我自己确认或修改的核心公式与接口
- **Mixer 矩阵 B 的每行**：自己核对 frames.hpp 的电机布局与旋向，把 τx 行定为 `dirY·(l/√2)kF`、τy 行定为 `-dirX·(l/√2)kF`、τz 行定为 `sgn·kM`。AI 初版 τy 符号方向我重新推了一遍叉乘再定稿。
- **roll/yaw 符号断言**：坚持给 mixer 加 “+roll → 左侧电机快”、”+yaw → CCW 电机快” 这两条物理断言（而非仅 round-trip），由我要求加入；这是发现 mixer 符号错的唯一信号。
- **四元数积分用 q⊗Ω 并每步归一化**（而非 Euler 角），由我确认这是 1 kHz 下的稳妥做法。
- 接口顺序 `motor_rpm_cmd = [FL, FR, BL, BR]`：与 frames.hpp `MotorIndex` 枚举绑定，禁止各节点硬编码。

## 5. AI 生成过的错误及修正
- **conda Python 抢占 colcon**：CMake 报 `No module named 'catkin_pkg'`，因为 colcon 调到 `miniconda3/envs/writ/bin/python3`。→ 剥离 PATH 到系统目录 + `--cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`，且这是每次 colcon build 都须带上的。
- **header-only 包无 imported target**：downstream `target_link_libraries(... drone_common::drone_common)` 报 `Target not found`。→ drone_common 建 INTERFACE library + `ament_export_targets(drone_common_targets)`，downstream 用同名命名空间 target。
- **本包测试链接 ALIAS**：`ament_add_gtest` 的目标链 `drone_common::drone_common` 报 target not found。→ 本包内测试链本地非命名空间 `drone_common`，仅 downstream 用命名空间版本。
- **git 首次 push `源引用规格 main 没有匹配`**：本地分支是 `master`。→ `git branch -M main`。
- **git HTTPS 推送 `could not read Username`**：非交互环境无法输入凭据。→ 改用 SSH（生成 ed25519 key + 加 GitHub）。
- **Assert 在 Release 被关掉**：在 mixer 加 `assert(k_F>0)` 等参数校验后，实测默认 Debug 构建会 abort 守住负参数（退出码 134），但 `-DNDEBUG`（Release/RelWithDebInfo）下 assert 被编译掉、默默放过错误。→ assert 只作 Debug 兜底，dynamics/controller 节点启动时还须显式 `RCLCPP_ERROR` 校验参数并退出（Step 2/3 落实）。常驻防炸（`wrenchToMotorSq` 的 `cwiseMax(0)` 非负 clamp）不依赖构建类型。

## 6. 如何验证动力学、控制器和 ROS2 topic 是否正确
- **drone_common**：`colcon test` 跑 7 条 gtest，验证 B·B⁻¹≈I、hover/roll/yaw/pitch 符号、四元数积分解析一致性、vee(skew) 逆。回归网随仓库进 git。
- **drone_dynamics（待 Step 2）**：零推力命令下应自由落体；给定 hover wrench 应保持悬停；`ros2 topic echo /drone/odom` 验帧率与 frame_id；rviz 见无人机 marker。
- **drone_controller（待 Step 3）**：悬停 (0,0,1.5) 稳态误差 < 0.3 m（plot_bag 出位置误差曲线当证据）。
- **ROS2 topic**：`ros2 topic list`/`info`/`echo` 逐个确认 topic 存在、类型与发布者订阅者对应；rosbag 录制 + plot_bag.py 复现为最终接受证据。
- 验证策略：每个模块单独 build → 针对性 echo/plot → bag 出曲线，不口头声称通过。

## 更新历史
- Step 0：占位骨架 + git（内容见 commit 6bf683b、5e36c28）
- Step 1：drone_common mixer/frames/math/obstacle + gtest 7 通过
- Step 2：drone_dynamics EOM + motor + odom/imu/path/tf（commit 870a8f5）
- Step 3：drone_controller cascaded PD + bringup(no_planner) 悬停（commit f8cfb2e）
- Step 4：go-to-goal + waypoint_sender 正方形航线（commit 9ac6eff）
- Step 5：drone_map 确定性障碍物 + MarkerArray（commit aa6d159）
- Step 6：drone_planner A* 膨胀栅格 + full.launch（commit aeac6ba）
- Step 7：plot_bag + report + ai_usage 收尾

## 7. Steps 2-6 新增经验

### AI 完成的模块（续）
- drone_dynamics 全部（dynamics.hpp, dynamics_node.cpp, dynamics.yaml）
- drone_controller 全部（controller.hpp, controller_node.cpp, controller.yaml, LADRC 实验）
- drone_map 全部（map_node.cpp, map.yaml）
- drone_planner 全部（grid.hpp, astar.hpp, planner_node.cpp, planner.yaml）
- launch 文件（no_planner.launch.py, full.launch.py）
- 辅助脚本（waypoint_sender.py, ladrc_autotune.py, plot_bag.py）

### 关键 prompt（续）
10. 「动力学离线自检：零推力自由落体、hover 保持、roll 符号」——确认 EOM/mixer/积分正确
11. 「单线程下 1kHz timer 饿死 RPM 订阅回调」→ 改为 MT executor + 独立 callback group
12. 「路径平滑共线简化穿过了球体」→ Brsenham lineFree 碰撞检查
13. 「YAML 的 string_array 参数在 launch 里不加载」→ PathJoinSubstitution→get_package_share_directory
14. 「跨 replan 的 path_idx_ 越界导致跳过航点」→ clamp path_idx_ 到新路径 bounds

### 自己确认/修改（续）
- **Roll 符号物理校验**：自己推了一遍 mixer τx 行的叉乘，确认「+roll→左电机快」物理正确
- **防炸机制**：要求加 assert(k_F>0) 参数校验 + cwiseMax(0) 非负 clamp + NDEBUG 陷阱记录
- **碰撞感知平滑**：要求 Bresenham 线检查而非仅几何共线判定
- **障碍物布局**：自己设计 6 个障碍物位置，确保目标点 free 但直线全 blocked
- **PD 调参**：自己调整 Kp/Kd 到 ζ≈0.7，后续又尝试过阻尼 (ζ≈1.1)

### 新增错误及修正（续）
- **1kHz timer 饿死订阅**：单线程 spin 下 1kHz wall_timer 占满 CPU，RPM 回调永远不触发。→ MT executor(2)+独立 MutuallyExclusive callback group。**这是我以前不知道的 ROS2 坑。**
- **ros2 topic echo --once 耗 1.5s**：每采一个数据点都重新启动 ros2 CLI。→ 改用 subprocess.Popen 后台管道持续读取
- **LADRC 三轴独立模型发散**：解耦 LADRC 不捕捉姿态内环 + 推力矢量耦合。→ PD 保持默认，LADRC 降为实验性 bonus（commit 12c2417）
# drone_common

共享头文件包（header-only）。dynamics 与 controller 共用同一份 mixer，杜绝符号错翻车。

## 提供的头
- `frames.hpp` — 世界系 `map`（ENU）、机体系 `base_link`、X 型 4 电机位置与旋向、`MotorIndex` 枚举（`[FL,FR,BL,BR]`）、物理常数。
- `mixer.hpp` — 控制分配矩阵 `B`（`[F,τx,τy,τz]ᵀ = B·[ω1²..ω4²]ᵀ`）及其逆 `B⁻¹`。
- `math.hpp` — 四元数一次积分（每步归一化）、skew、vee、由“机体期望 z + 偏航”构 `R_des`（避开 Euler 奇异）。
- `obstacle.hpp` — `Obstacle{Sphere/Cylinder/Cube}` 结构、2D 膨胀 footprint、z 跨度判定、点到障碍距离。
- `drone_common.hpp` — 聚合 include。

## 单元测试
`colcon test --packages-select drone_common` 运行 7 条 gtest：
- `Mixer.BInverseIsIdentity` / `HoverAllEqual` / `RollTorqueSign` / `YawTorqueSign`
- `Math.QuaternionIntegrateZAxis` / `RotationFromZAndYaw` / `VeeSkewInverse`

roll/yaw 符号断言是“无人机不翻车”的物理校验：改动 mixer 时若这两条报红，必有方向错。

## 关键约定
- `/drone/motor_rpm_cmd` (Float32MultiArray) 顺序 `[FL, FR, BL, BR]`，单位 RPM。
- 旋向 `kMotorYawSign = [+1,-1,-1,+1]`（FL/BR = CCW，FR/BL = CW）。
- 所有节点取电机顺序/符号必须 include `frames.hpp`，禁止重新硬编码。
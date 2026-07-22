# 局部导航参数

配置文件：`src/local_planner/launch/local_planner.launch`

文件顶部的 `<arg>` 可以通过 `ros2 launch ... name:=value` 覆盖；其余 `<param>`
需要修改 launch 文件，或由上层 launch 显式转发。

## 运行速度

| 参数 | 默认值 | 单位 | 说明 |
|---|---:|---|---|
| `maxSpeed` | 1.0 | m/s | 导航平移合速度上限，不是硬件最大值 |
| `autonomySpeed` | 1.0 | m/s | 自动模式期望速度，建议不大于 `maxSpeed` |
| `maxVx` / `minVx` | 3.0 / 0.05 | m/s | 前向轴最大值和最小非零值 |
| `maxVy` / `minVy` | 1.0 / 0.10 | m/s | 侧向轴最大值和最小非零值 |
| `maxYawRate` / `minYawRate` | 3.0 / 0.02 | rad/s | yaw 轴最大值和最小非零值 |
| `maxAccel` | 1.0 | m/s² | 平移非零命令之间的变化限制 |
| `maxYawAccel` | 3.0 | rad/s² | yaw 非零命令之间的变化限制 |

SDK 合法输出为：

- `vx = 0` 或 `|vx| ∈ [0.05, 3.0] m/s`；
- `vy = 0` 或 `|vy| ∈ [0.10, 1.0] m/s`；
- `yaw_rate = 0` 或 `|yaw_rate| ∈ [0.02, 3.0] rad/s`。

节点启动时会检查这些硬件参数，越界配置会直接报错退出。

建议初次真机调试使用：

```text
maxSpeed       = 0.5 ~ 0.8 m/s
autonomySpeed  = 0.4 ~ 0.6 m/s
maxYawRate     = 0.8 ~ 1.5 rad/s
```

`maxYawRate` 的 SDK 硬上限是 3.0，但真机不一定适合长期使用该速度。

## 机器人足迹和障碍物

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `vehicleLength` | 0.7 m | 机体前后长度 |
| `vehicleWidth` | 0.4 m | 机体左右宽度 |
| `safetyMargin` | 0.10 m | 矩形四周额外膨胀距离 |
| `checkObstacle` | true | 检查平移碰撞 |
| `checkRotObstacle` | true | 检查旋转扫掠碰撞 |
| `pointPerPathThre` | 2 | 足迹内达到该障碍点数后判定阻塞 |
| `adjacentRange` | 2.5 m | 点云裁剪和局部规划半径 |
| `obstacleHeightThre` | 0.15 | terrain intensity 高于该值视为硬障碍 |
| `groundHeightThre` | 0.10 | terrain intensity 低于该值视为普通地面 |

`useTerrainAnalysis=true` 时使用 `/terrain_map` 的 intensity；设为 false 时使用
`/registered_scan`，并通过 `minRelZ/maxRelZ` 过滤相对高度。

安全边距至少应覆盖定位误差、点云误差、机身尺寸误差和摆腿范围。狭窄环境可以逐步从
`0.10 m` 降至 `0.05 m`，不建议直接设为零。

## 候选路径

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `candidateAngleStep` | 5 deg | 全圆候选方向间隔 |
| `pathResolution` | 0.10 m | 每条候选的碰撞采样间隔 |
| `minCandidateLength` | 0.30 m | 可执行候选的最小安全长度 |
| `minPathRange` | 0.50 m | 低速时最小规划范围 |
| `pathRangeBySpeed` | true | 根据当前速度比例缩放规划范围 |

减小角度间隔或路径分辨率会提高检测精度，同时增加 CPU 开销。

## 路径评分

| 参数 | 默认值 | 增大后的影响 |
|---|---:|---|
| `goalWeight` | 4.0 | 更重视接近目标 |
| `headingWeight` | 0.8 | 更偏好目标方向 |
| `lengthWeight` | 1.5 | 更偏好较长安全路径 |
| `terrainCostWeight` | 1.0 | 更回避高代价地形 |
| `directionChangeWeight` | 0.25 | 减少左右方向来回切换 |
| `traversalTimeWeight` | 0.4 | 更偏好执行时间较短的方向 |

`maxForwardVelocity=3.0` 和 `maxLateralVelocity=1.0` 只用于估算候选执行时间；最终输出
仍由跟随器的速度参数限制。

## 跟随与到点

| 参数 | 默认值 | 说明 |
|---|---:|---|
| `lookAheadDis` | 0.25 m | 路径跟随前视距离 |
| `posGain` | 4.0 | 平移位置比例增益 |
| `yawGain` | 2.0 | 最终 Pose yaw 增益 |
| `pathYawGain` | 3.0 | 路径执行阶段 yaw 增益 |
| `slowDwnDisThre` | 1.0 m | 到目标前开始减速的距离 |
| `poseServoDisThre` | 1.5 m | 允许切换到最终 Pose servo 的距离 |
| `axisStopPosThre` | 0.03 m | 单轴误差低于该值时该轴回零 |
| `goalPosThre` | 0.20 m | Goal 位置完成阈值 |
| `goalYawThre` | 10 deg | Goal yaw 完成阈值 |

由于 `vy` 的最小非零速度为 `0.1 m/s`，位置阈值过小可能引起频繁修正。建议先用
`0.10~0.20 m` 完成真机验证，再根据定位噪声逐步收紧。

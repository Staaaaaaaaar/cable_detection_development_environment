# Topic、坐标系与运行调试

## Topic 接口

### localPlanner 订阅

| Topic | 类型 | 作用 |
|---|---|---|
| `/state_estimation` | `nav_msgs/msg/Odometry` | 当前机器人位姿 |
| `/goal_pose` | `geometry_msgs/msg/PoseStamped` | Goal Pose，约定为 `map` frame |
| `/terrain_map` | `sensor_msgs/msg/PointCloud2` | 地形分析开启时的障碍和代价输入 |
| `/registered_scan` | `sensor_msgs/msg/PointCloud2` | 地形分析关闭时的点云输入 |
| `/navigation_boundary` | `geometry_msgs/msg/PolygonStamped` | 禁止越过的导航边界 |
| `/added_obstacles` | `sensor_msgs/msg/PointCloud2` | 外部注入的障碍物 |
| `/check_obstacle` | `std_msgs/msg/Bool` | 动态开关障碍检测 |
| `/speed` | `std_msgs/msg/Float32` | 外部期望速度 |
| `/joy` | `sensor_msgs/msg/Joy` | 手动/自动模式输入 |

### localPlanner 发布

| Topic | 类型 | 作用 |
|---|---|---|
| `/path` | `nav_msgs/msg/Path` | `vehicle` frame 下的局部安全路径 |
| `/free_paths` | `sensor_msgs/msg/PointCloud2` | RViz 显示候选路径 |

### pathFollower 订阅与发布

跟随器订阅 `/state_estimation`、`/goal_pose`、`/path`、`/speed`、`/joy` 和 `/stop`，
发布 `geometry_msgs/msg/Twist` 类型的 `/cmd_vel`。

`/stop` 为 `std_msgs/msg/Int8`：

- `0`：正常运行；
- `1`：停止 `vx/vy`，允许 yaw；
- `2` 或更大：停止所有速度轴。

## 坐标系约定

- `/state_estimation` 和 `/goal_pose` 的位置必须处于同一 `map` 坐标系；
- `/terrain_map`、`/registered_scan`、边界和附加障碍物必须与定位使用同一世界坐标系；
- `/path` 使用接收该路径时刻的 `vehicle` 局部坐标系；
- `/cmd_vel` 是机器狗机体系速度：`x` 向前、`y` 向左、`z` 轴正角速度逆时针。

当前节点不使用 TF 自动转换 Goal 或点云。frame 不一致会产生方向或位置错误，但不一定
触发显式异常，因此接入新定位系统时应优先核对 frame。

## 构建

当前工作区只保留局部导航闭环、仿真传感器和 RViz Goal 工具。`waypoint_rviz_plugin`
虽然不参与控制计算，但默认 RViz 配置直接使用它发送 `/goal_pose`，因此是运行依赖。

完整仿真构建：

```bash
./scripts/build.sh
```

真机依赖构建：

```bash
./scripts/build_real_robot.sh
```

如果仓库移动过路径，旧 `build/` 中的 CMake cache 可能仍引用原绝对路径。可以使用新的
构建目录验证，而不覆盖原缓存：

```bash
colcon build \
  --build-base build_current \
  --install-base install_current \
  --symlink-install \
  --packages-select local_planner vehicle_simulator
```

## 启动

单独启动局部规划：

```bash
source install/setup.bash
ros2 launch local_planner local_planner.launch
```

真机组合启动：

```bash
./scripts/launch_real_robot_local_nav.sh \
  use_rviz:=true maxSpeed:=0.8 autonomySpeed:=0.6 \
  goalPosThre:=0.15 goalYawThre:=5.0
```

## 检查清单

```bash
ros2 topic hz /state_estimation
ros2 topic hz /terrain_map
ros2 topic echo /goal_pose --once
ros2 topic echo /path --once
ros2 topic echo /cmd_vel
```

应确认：

1. 定位和点云持续更新且时间戳前进；
2. Goal frame 为 `map`；
3. 空旷区域 `/path` 至少有两个点；
4. 无安全路径时 `/path` 只有一个点且 `/cmd_vel` 全零；
5. 所有非零速度均满足 SDK 合法区间；
6. RViz 中 Box 膨胀范围与机器狗真实外形、摆腿范围一致。

## 常见现象

### 一直没有路径

- 检查 `useTerrainAnalysis` 与实际输入 topic 是否匹配；
- 检查 terrain intensity 是否符合 `groundHeightThre/obstacleHeightThre`；
- 检查点云和定位是否在同一坐标系；
- 临时观察 `/free_paths`，不要直接关闭障碍检测用于真机运行。

### 接近目标时横向抖动

- 适当增大 `goalPosThre` 或 `axisStopPosThre`；
- 降低 `posGain`、`autonomySpeed`；
- 检查定位横向噪声是否超过完成阈值。

### 转向过快

SDK 允许 `3 rad/s` 不代表真机应采用该速度。降低 `maxYawRate` 和 `pathYawGain`，并根据
实际机身惯量调整 `maxYawAccel`。

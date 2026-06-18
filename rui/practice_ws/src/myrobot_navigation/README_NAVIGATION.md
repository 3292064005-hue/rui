# 自主导航说明

本文说明本包的 ROS1 自主导航配置，包括静态地图加载、AMCL 定位、move_base 路径规划和自动目标点巡航。

---

## 1. 导航架构

```text
Gazebo 原始世界
  ↓
机器人仿真驱动：/cmd_vel、/odom、/scan、TF
  ↓
map_server 加载静态地图
  ↓
AMCL 使用 /scan + /odom 进行定位
  ↓
move_base 使用 GlobalPlanner + TebLocalPlannerROS 进行全局路径规划和全向局部避障
  ↓
move_base_waypoint_navigator.py 顺序发送目标点
  ↓
识别节点和证据记录节点输出任务结果
```

默认导航地图为最终 SLAM 扫描结果：

```text
myrobot_navigation/maps/raicom_slam_map_final.pgm
myrobot_navigation/maps/raicom_slam_map_final.yaml
```

该地图用于 `map_server`。Gazebo 世界中的识别面板仅参与视觉渲染，
不参与碰撞和激光扫描，因此不会改变地图可通行结构。

---

## 2. 依赖安装

```bash
sudo apt install \
  ros-$ROS_DISTRO-map-server \
  ros-$ROS_DISTRO-amcl \
  ros-$ROS_DISTRO-move-base \
  ros-$ROS_DISTRO-navigation \
  ros-$ROS_DISTRO-gazebo-plugins
```

---

## 3. 启动导航栈，手动给目标

```bash
cd /root/workspace/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch myrobot_navigation navigation.launch
```

启动后，在 RViz 中使用：

```text
2D Pose Estimate   # 校正初始位姿
2D Nav Goal        # 手动发送目标点
```

---

## 4. 一键自主导航任务

```bash
roslaunch myrobot_navigation autonomous_navigation.launch
```

等价的默认任务入口是：

```bash
roslaunch myrobot_task task_patrol.launch
```

当前仓库已将 `task_patrol.launch` 切换为导航栈版本，方便直接按比赛任务入口启动。

该启动文件会自动执行：

1. 加载 Gazebo 原始世界；
2. 生成机器人；
3. 启动雷达、相机、IMU 和轮子仿真；
4. 启动 `map_server`；
5. 启动 `amcl`；
6. 启动 `move_base`；
7. 发布初始位姿；
8. 顺序发送导航目标点；
9. 到达识别区后输出识别结果；
10. 保存轨迹和任务总结。

---

## 5. 关键话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/map` | `nav_msgs/OccupancyGrid` | 静态地图 |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL 定位结果 |
| `/move_base/status` | `actionlib_msgs/GoalStatusArray` | 导航目标状态 |
| `/move_base/global_costmap/costmap` | `nav_msgs/OccupancyGrid` | 全局代价地图 |
| `/move_base/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | 局部代价地图 |
| `/navigation_status` | `std_msgs/String` | 自动导航状态 |
| `/navigation_health` | `std_msgs/String` | 导航健康状态 |
| `/recognition_result` | `std_msgs/String` | 单个识别区结果 |
| `/recognition_summary` | `std_msgs/String` | 识别汇总结果 |

---

## 6. 导航目标点配置

导航目标点文件：

```text
myrobot_navigation/config/navigation_params.yaml
```

当前正式路线：

```yaml
navigation_goals:
  - {name: zone_1, x: 0.52, y: -2.55, yaw:  0.0000, hold: 1.5}
  - {name: right_wall_exit, x: 3.00, y: -2.00, yaw: -3.1416, hold: 0.1}
  - {name: finish, x: 0.30, y: -0.08, yaw:  3.1416, hold: 0.1}
```

正式巡航默认只保留完整路线中的第 3、6、13 个导航点：
`zone_1`、`right_wall_exit`、`finish`。
程序内部会把长路径切成更小的 move_base 子目标，减少 TEB 一次性无可行轨迹的概率。
这些内部过渡点配置在 `navigation_internal_route`，不会改变正式导航点数量。
建图覆盖路线单独维护在 `myrobot_navigation/config/slam_navigation_params.yaml`。

字段说明：

| 字段 | 说明 |
|---|---|
| `name` | 目标点名称，日志中会显示 |
| `x`、`y` | 地图坐标，单位 m |
| `yaw` | 目标朝向，单位 rad |
| `hold` | 到达后停留时间，单位 s |

正式巡航由 GlobalPlanner 和 TebLocalPlannerROS 规划路径、局部避障并直接发布 `/cmd_vel`。
`cmd_vel_target_yaw_filter.py` 不再插入正式导航的速度指令链路，因此不会额外强制速度方向或车头朝向。
底盘按全向/麦克纳姆模型处理：车头朝向和速度方向解耦，任意地图方向的移动都可以按当前车头分解为前进和横移。
因此 x/y 平移能力保持对称，`myrobot_navigation/config/base_local_planner_params.yaml` 中 `max_vel_x == max_vel_y`，
`acc_lim_x == acc_lim_y`，不能通过压低横移速度来解决轨迹问题。

当前 TEB 配置重点约束局部轨迹稳定性：

- `max_global_plan_lookahead_dist: 1.20`，避免局部规划只看很短一段路；
- `global_plan_viapoint_sep: 0.08`，`weight_viapoint: 90.0`，强制局部轨迹贴近全局路径；
- `min_obstacle_dist: 0.08`，`inflation_dist: 0.30`，`weight_obstacle: 220.0`，`weight_inflation: 8.0`，保持离墙余量但不把狭窄通道判死；
- `weight_optimaltime: 0.01`，降低为了抢时间而横向摆动的倾向；
- `yaw_goal_tolerance: 3.14`，避免 TEB 在终点强制处理车头朝向。

本地代价地图由 `myrobot_navigation/config/local_costmap_common_params.yaml` 配置，只使用过滤后的激光障碍层做短距离避障。
静态地图墙体留在全局代价地图中，TEB 通过高权重 via-points 贴合全局路径。
不要把静态地图层直接放进 rolling local costmap；TEB 会把大量膨胀地图格当作点障碍，
在起点或窄通道附近容易直接报 `trajectory is not feasible`。
`navigation.launch` 中的 `scan_self_filter.py` 默认过滤 0.45m 以内的雷达点，避免车体/轮子自扫残留污染本地代价地图。
正式巡航和建图巡航都保留必要的中间过渡点，减少单个目标过长导致的全局路径失败。

---

## 7. 使用 SLAM 保存的地图导航

如果已经通过 SLAM 生成地图：

```bash
roslaunch myrobot_navigation slam_mapping.launch
roslaunch myrobot_navigation save_slam_map.launch
```

则可以指定地图文件启动自主导航：

```bash
roslaunch myrobot_navigation autonomous_navigation.launch \
  map_file:=$HOME/myrobot_navigation_maps/raicom_slam_map.yaml
```

如果定位偏差较大，在 RViz 中重新点击 `2D Pose Estimate`。

---

## 8. 常用启动参数

### 8.1 `navigation.launch`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map_file` | `myrobot_navigation/maps/raicom_slam_map_final.yaml` | map_server 加载的地图 |
| `launch_rviz` | `true` | 是否打开 RViz |
| `gui` | `true` | 是否显示 Gazebo GUI |
| `paused` | `false` | Gazebo 是否暂停启动 |
| `initial_pose_x` | `0.0` | AMCL 初始 x |
| `initial_pose_y` | `0.0` | AMCL 初始 y |
| `initial_pose_a` | `-1.5708` | AMCL 初始朝向 |
| `auto_initial_pose` | `true` | 是否自动发布初始位姿 |

### 8.2 `autonomous_navigation.launch`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `start_goals` | `true` | 是否自动发送目标点 |
| `start_recognition` | `true` | 是否启动识别节点 |
| `record_evidence` | `true` | 是否保存任务证据 |
| `launch_rviz` | `true` | 是否打开 RViz |

---

## 9. 常见问题

### 9.1 `/map` 没有数据

```bash
rosnode list | grep map_server
rostopic echo /map -n 1
```

如果地图路径错误，检查 `map_file:=...` 是否指向存在的 `.yaml` 文件。

### 9.2 `/amcl_pose` 没有数据

检查：

```bash
rostopic echo /scan -n 1
rostopic echo /odom -n 1
rosrun tf tf_echo odom base_footprint
```

AMCL 依赖雷达、里程计和 TF。

### 9.3 `move_base` 不规划或不动

检查：

```bash
rostopic echo /move_base/status
rostopic echo /move_base/global_costmap/costmap -n 1
rostopic echo /move_base/local_costmap/costmap -n 1
```

常见原因：

- 初始位姿不准；
- 目标点在障碍物里；
- 目标点离墙太近；
- 局部代价地图膨胀把狭窄通道堵住；
- `/scan` 或 TF 异常。

### 9.4 局部路径抖动、贴墙或卡住

优先调：

```text
myrobot_navigation/config/navigation_params.yaml
myrobot_navigation/config/base_local_planner_params.yaml
myrobot_navigation/config/local_costmap_common_params.yaml
```

建议：

- 先确认 RViz 中 `/move_base/GlobalPlanner/plan` 没有穿墙；
- 检查 `/move_base/TebLocalPlannerROS/local_plan` 是否大幅偏离全局路径；
- 保持 `max_vel_x/max_vel_y` 和 `acc_lim_x/acc_lim_y` 对称，符合全向底盘模型；
- 如果黄色局部路径往墙边摆，优先增大 `weight_viapoint` 或检查 `/scan_filtered` 是否仍有车体自扫近距离点；
- 如果通道被膨胀层堵住，再小幅降低 `inflation_radius` 或检查目标点是否贴墙。

---

## 10. 地图文件

默认导航使用巡航扫描后保存的地图：

```text
src/myrobot_navigation/maps/raicom_slam_map_final.pgm
src/myrobot_navigation/maps/raicom_slam_map_final.yaml
```

`raicom_known_map.*` 仅作为世界几何参考地图保留，不是默认导航输入。
需要重新建图时运行 `slam_mapping.launch`，完成巡航后再运行
`save_slam_map.launch`。

---

## 11. 验收点

自主导航功能正常时，应满足：

1. `/map` 有数据；
2. `/amcl_pose` 有数据；
3. `/move_base/status` 有状态；
4. RViz 中能看到全局路径和局部代价地图；
5. 自动导航节点能依次发送目标；
6. 机器人能到达识别区和终点；
7. 任务结束后能生成日志和总结。

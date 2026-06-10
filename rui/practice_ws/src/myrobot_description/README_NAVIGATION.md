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
move_base 进行全局路径规划和局部避障
  ↓
move_base_waypoint_navigator.py 顺序发送目标点
  ↓
识别节点和证据记录节点输出任务结果
```

默认导航地图为最终 SLAM 扫描结果：

```text
maps/raicom_slam_map_final.pgm
maps/raicom_slam_map_final.yaml
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
roslaunch myrobot_description navigation.launch
```

启动后，在 RViz 中使用：

```text
2D Pose Estimate   # 校正初始位姿
2D Nav Goal        # 手动发送目标点
```

---

## 4. 一键自主导航任务

```bash
roslaunch myrobot_description autonomous_navigation.launch
```

等价的默认任务入口是：

```bash
roslaunch myrobot_description task_patrol.launch
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
config/navigation_params.yaml
```

精简示例：

```yaml
navigation_goals:
  - {name: left_wall_exit, x: 0.00, y: -3.50, yaw: -1.5708, hold: 0.1}
  - {name: zone_1,         x: 0.52, y: -2.55, yaw:  0.0000, hold: 0.1}
  - {name: zone_2,         x: 4.45, y: -1.65, yaw:  3.1416, hold: 0.1}
  - {name: finish,         x: 0.00, y:  0.00, yaw:  3.1416, hold: 0.1}
```

完整的 12 个点只在 `config/navigation_params.yaml` 维护。

字段说明：

| 字段 | 说明 |
|---|---|
| `name` | 目标点名称，日志中会显示 |
| `x`、`y` | 地图坐标，单位 m |
| `yaw` | 目标朝向，单位 rad |
| `hold` | 到达后停留时间，单位 s |

建议保留中间过渡点，不要让机器人一次性跨越复杂通道或墙体。

---

## 7. 使用 SLAM 保存的地图导航

如果已经通过 SLAM 生成地图：

```bash
roslaunch myrobot_description slam_mapping.launch
roslaunch myrobot_description save_slam_map.launch
```

则可以指定地图文件启动自主导航：

```bash
roslaunch myrobot_description autonomous_navigation.launch \
  map_file:=$HOME/myrobot_description_maps/raicom_slam_map.yaml
```

如果定位偏差较大，在 RViz 中重新点击 `2D Pose Estimate`。

---

## 8. 常用启动参数

### 8.1 `navigation.launch`

| 参数 | 默认值 | 说明 |
|---|---|---|
| `map_file` | `maps/raicom_slam_map_final.yaml` | map_server 加载的地图 |
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
- 局部代价地图膨胀过大；
- `/scan` 或 TF 异常。

### 9.4 机器人蹭墙或卡住

优先调：

```text
config/navigation_params.yaml
config/base_local_planner_params.yaml
config/costmap_common_params.yaml
```

建议：

- 增加中间目标点；
- 降低最大速度；
- 避免目标点贴墙；
- 使用 RViz 检查 costmap 是否把通道堵住。

---

## 10. 地图文件

默认导航使用巡航扫描后保存的地图：

```text
src/myrobot_description/maps/raicom_slam_map_final.pgm
src/myrobot_description/maps/raicom_slam_map_final.yaml
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

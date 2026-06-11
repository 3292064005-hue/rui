# SLAM 建图说明

本文说明如何在 Gazebo 比赛地图中使用激光和里程计建图，并保存生成的栅格地图。

---

## 1. 当前默认方案

默认建图后端是 `slam_toolbox`：

```text
Gazebo 原始世界
  ↓
/scan -> scan_self_filter.py -> /scan_filtered
  ↓
/odom + TF + /scan_filtered
  ↓
slam_toolbox 图优化建图和回环修正
  ↓
/map
  ↓
save_slam_map.py 保存 .pgm + .yaml
```

选择 `slam_toolbox` 的原因：

- 比 `gmapping` 更适合 2D 激光 + 里程计的 pose graph 建图；
- 支持回环检测和图优化，重复走廊里比纯 scan matching 更稳；
- ROS1 Noetic 有二进制包，工程可直接通过 launch 集成；
- 可切换同步/异步模式，并保留地图序列化能力。

当前 `slam_mapping.launch` 默认：

```text
mapping_backend:=slam_toolbox
slam_toolbox_mode:=async
```

异步模式在当前 Gazebo 自动巡航建图中更稳定。需要对比同步节点时可改为：

```bash
roslaunch myrobot_description slam_mapping.launch slam_toolbox_mode:=sync
```

---

## 2. 依赖安装

```bash
sudo apt install \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-slam-gmapping \
  ros-$ROS_DISTRO-map-server
```

如果需要键盘手动控制：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

---

## 3. 一键自动建图

```bash
cd ~/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch myrobot_description slam_mapping.launch
```

默认会启动：

1. Gazebo 默认比赛世界；
2. 机器人模型和仿真驱动；
3. `/scan` 激光雷达；
4. `scan_self_filter.py` 输出 `/scan_filtered`；
5. `slam_toolbox` 输出 `/map`；
6. RViz 建图显示；
7. 自动巡点节点；
8. SLAM 状态监控和轨迹记录。

自动建图读取 `config/slam_navigation_params.yaml` 中的
`navigation_goals`，保留比正式巡航更密的 12 点路线。建图阶段尚无
静态地图，因此巡航节点使用 `odom` 坐标逐点平移，不依赖 `move_base`
或 `/move_base/make_plan`。默认完整巡航 3 次，用重复观测帮助回环和优化：

```bash
roslaunch myrobot_description slam_mapping.launch patrol_repeats:=3
```

---

## 4. 后端切换

默认推荐：

```bash
roslaunch myrobot_description slam_mapping.launch mapping_backend:=slam_toolbox
```

保留两个对比后端：

```bash
roslaunch myrobot_description slam_mapping.launch mapping_backend:=odom_laser
roslaunch myrobot_description slam_mapping.launch mapping_backend:=gmapping
```

用途区别：

| 后端 | 用途 |
|---|---|
| `slam_toolbox` | 默认最优方案，pose graph、回环、Ceres 优化 |
| `odom_laser` | 仿真专用固定边界投影，用于快速生成干净几何参考图 |
| `gmapping` | 传统 scan matching 对比基线 |

`slam_toolbox` 参数文件：

```text
config/slam_toolbox_params.yaml
```

`gmapping` 参数文件：

```text
config/slam_gmapping_params.yaml
```

---

## 5. 手动控制建图

如果想自己控制机器人运动：

```bash
roslaunch myrobot_description slam_mapping.launch autonomous_mapping:=false
```

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source ~/practice_ws/devel/setup.bash
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

麦克纳姆底盘支持：

```text
linear.x   前后
linear.y   横移
angular.z  旋转
```

---

## 6. 保存地图

当 RViz 中 `/map` 基本完整后，另开终端执行：

```bash
source /opt/ros/noetic/setup.bash
source ~/practice_ws/devel/setup.bash
roslaunch myrobot_description save_slam_map.launch
```

默认保存到：

```text
~/myrobot_description_maps/raicom_slam_map.pgm
~/myrobot_description_maps/raicom_slam_map.yaml
```

指定保存目录和名称：

```bash
roslaunch myrobot_description save_slam_map.launch \
  output_dir:=$HOME/my_maps \
  map_name:=raicom_final_map
```

仓库中用于导航的验收地图为：

```text
src/myrobot_description/maps/raicom_slam_map_final.pgm
src/myrobot_description/maps/raicom_slam_map_final.yaml
```

---

## 7. 关键话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | Gazebo 原始激光 |
| `/scan_filtered` | `sensor_msgs/LaserScan` | 建图使用的自车近场过滤激光 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM 输出地图 |
| `/slam_status` | `std_msgs/String` | 建图状态 JSON |
| `/task_trajectory` | `nav_msgs/Path` | 自动建图轨迹 |

---

## 8. 重要 TF 链路

SLAM 需要以下坐标关系正常：

```text
map -> odom -> base_footprint -> base_link -> base_scan
```

检查命令：

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_link base_scan
```

如果 TF 不通，`slam_toolbox` 无法稳定输出地图。

---

## 9. 常见问题

### 9.1 RViz 中没有地图

检查：

```bash
rosnode list | grep slam_toolbox
rostopic echo /scan_filtered -n 1
rostopic echo /map -n 1
```

如果 `/scan_filtered` 没有数据，优先检查 Gazebo 雷达插件和
`scan_self_filter.py`。

### 9.2 地图变形或重影

优先调 `config/slam_toolbox_params.yaml`：

```yaml
minimum_travel_distance: 0.08
minimum_travel_heading: 0.12
loop_match_minimum_response_fine: 0.45
correlation_search_space_dimension: 0.60
```

如果机器人运动过快，也可降低 `slam_mapping.launch` 中自动巡航速度参数。

### 9.3 保存地图失败

先确认 `/map` 已经有数据：

```bash
rostopic echo /map -n 1
```

再执行：

```bash
roslaunch myrobot_description save_slam_map.launch
```

### 9.4 为什么识别图板不影响建图

识别图板是 `visual-only` 物体，没有 collision。这样摄像头可以看到目标卡，
但激光、代价地图和 SLAM 边界保持不变。

---

## 10. 验收点

SLAM 功能正常时，应满足：

1. `/scan_filtered` 有数据；
2. `/odom` 有数据；
3. `tf_echo odom base_footprint` 正常；
4. `rosnode list` 能看到 `/slam_toolbox`；
5. RViz 中能看到 `/map` 逐渐生成；
6. `save_slam_map.launch` 能生成 `.pgm` 和 `.yaml` 文件。

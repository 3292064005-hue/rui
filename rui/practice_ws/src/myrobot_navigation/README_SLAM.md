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
slam_toolbox 稳定建图配置，使用 /odom + /scan_filtered 输出栅格地图
  ↓
/map
  ↓
save_slam_map.py 保存 .pgm + .png + .yaml
```

选择 `slam_toolbox` 的原因：

- 保留 `slam_toolbox` 建图链路，不再手工拟合墙线，开放口不容易被后处理补死；
- 当前仿真里 odom 稳定，默认关闭 scan matching 概率搜索，避免 karto 内部崩溃；
- 2 cm 分辨率下细节更自然，适合作为正式巡航地图来源。

当前 `slam_mapping.launch` 默认：

```text
mapping_backend:=slam_toolbox
```

---

## 2. 依赖安装

```bash
sudo apt install \
  ros-$ROS_DISTRO-slam-gmapping \
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-map-server
```

如果需要键盘手动控制：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

---

## 3. 一键自动建图

```bash
cd /home/chen/ros1_ultrasound_ws/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch myrobot_navigation slam_mapping.launch
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

自动建图读取 `myrobot_navigation/config/slam_navigation_params.yaml` 中的
`navigation_goals`，保留比正式巡航更密的 12 点路线。建图阶段尚无
静态地图，因此巡航节点使用 `odom` 坐标逐点平移，不依赖 `move_base`
或 `/move_base/make_plan`。当前规定默认完整巡航 1 次：

```bash
roslaunch myrobot_navigation slam_mapping.launch patrol_repeats:=1
```

---

## 4. 后端切换

默认推荐（`slam_toolbox`）：

```bash
roslaunch myrobot_navigation slam_mapping.launch
```

保留两个对比/备用后端：

```bash
roslaunch myrobot_navigation slam_mapping.launch mapping_backend:=odom_laser
roslaunch myrobot_navigation slam_mapping.launch mapping_backend:=gmapping
```

用途区别：

| 后端 | 用途 |
|---|---|
| `slam_toolbox` | 默认方案，稳定优先，使用 /odom + /scan_filtered 建高分辨率图 |
| `odom_laser` | 规则比赛场地专用备用方案，固定边界与轴线投影 |
| `gmapping` | 传统 scan matching 对比基线 |

`slam_toolbox` 参数文件：

```text
myrobot_navigation/config/slam_toolbox_params.yaml
```

`gmapping` 参数文件：

```text
myrobot_navigation/config/slam_gmapping_params.yaml
```

---

## 5. 手动控制建图

如果想自己控制机器人运动：

```bash
roslaunch myrobot_navigation slam_mapping.launch autonomous_mapping:=false
```

另开终端：

```bash
source /opt/ros/noetic/setup.bash
source /home/chen/ros1_ultrasound_ws/rui/rui/practice_ws/devel/setup.bash
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
source /home/chen/ros1_ultrasound_ws/rui/rui/practice_ws/devel/setup.bash
roslaunch myrobot_navigation save_slam_map.launch
```

默认保存到：

```text
~/myrobot_navigation_maps/raicom_slam_map.pgm
~/myrobot_navigation_maps/raicom_slam_map.yaml
```

指定保存目录和名称：

```bash
roslaunch myrobot_navigation save_slam_map.launch \
  output_dir:=$HOME/my_maps \
  map_name:=raicom_final_map
```

仓库中用于导航的验收地图为：

```text
src/myrobot_navigation/maps/raicom_slam_map_one_lap_test.pgm
src/myrobot_navigation/maps/raicom_slam_map_one_lap_test.yaml
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

如果 TF 不通，`odom_laser_mapper` 无法把激光正确投影到 `/map`。

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

若使用备用后端 `odom_laser`，则检查：

```bash
rosnode list | grep odom_laser_mapper
```

### 9.2 地图变形或重影

默认后端 `slam_toolbox` 优先调参数文件：

```text
myrobot_navigation/config/slam_toolbox_params.yaml
```

若使用备用后端 `odom_laser`，可调 `slam_mapping.launch` 中 `odom_laser_mapper.py` 的建图参数：

```xml
<param name="min_insert_translation" value="0.035" />
<param name="min_insert_rotation" value="0.08" />
<param name="max_gap_cells" value="7" />
<param name="axis_angle_tolerance" value="0.10" />
```

如果机器人运动过快，也可降低 `slam_mapping.launch` 中自动巡航速度参数。

### 9.3 保存地图失败

先确认 `/map` 已经有数据：

```bash
rostopic echo /map -n 1
```

再执行：

```bash
roslaunch myrobot_navigation save_slam_map.launch
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
4. 默认后端 `slam_toolbox` 正常运行（若使用 `odom_laser` 则确认 `/odom_laser_mapper` 存在）；
5. RViz 中能看到 `/map` 逐渐生成；
6. `save_slam_map.launch` 能生成 `.pgm` 和 `.yaml` 文件。

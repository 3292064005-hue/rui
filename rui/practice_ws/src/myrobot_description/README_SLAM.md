# SLAM 建图说明

本文说明如何在 Gazebo 比赛地图中使用激光和里程计建图，并保存生成的栅格地图。

---

## 1. 功能目标

SLAM 模式直接使用默认 `worlds/rm_map.world`。世界中的识别面板是纯视觉元素，不带碰撞，因此不会改变激光建图边界。流程如下：

```text
Gazebo 原始世界
  ↓
机器人运动 + /scan + /odom + TF
  ↓
odom_laser_mapper 默认建图
  ↓
/map
  ↓
保存为 .pgm + .yaml
```

---

## 2. 依赖安装

```bash
sudo apt install ros-$ROS_DISTRO-slam-gmapping ros-$ROS_DISTRO-map-server
```

如果需要键盘手动控制：

```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
```

---

## 3. 一键自动建图

```bash
cd ~/practice_ws
catkin_make
source devel/setup.bash
roslaunch myrobot_description slam_mapping.launch
```

默认会启动：

1. Gazebo 默认比赛世界；
2. 机器人模型和仿真驱动；
3. `/scan` 激光雷达；
4. `/odom` 里程计；
5. 默认 `odom_laser_mapper`（可选 `slam_gmapping`）；
6. RViz 建图显示；
7. 自动巡点节点；
8. SLAM 状态监控。

自动建图直接读取 `config/navigation_params.yaml` 中的
`navigation_goals`。此时尚无静态地图，因此节点使用 `odom` 坐标逐点
平移，不依赖 `move_base` 或 `/move_base/make_plan`。默认完整巡航 3 次，
用重复观测补齐墙体并过滤瞬时噪点。需要临时修改次数时可执行：

```bash
roslaunch myrobot_description slam_mapping.launch patrol_repeats:=3
```

默认 `mapping_backend:=odom_laser` 使用 Gazebo 里程计和激光构建固定边界栅格，
墙体需要至少 3 次激光命中才会进入地图，再经过小连通域过滤和正交墙体
线段提取。机器人实际走过的底盘区域会强制标记为空闲，避免近场自反射沿
巡航轨迹累计成假墙。

需要对比传统 gmapping 时可执行：

```bash
roslaunch myrobot_description slam_mapping.launch mapping_backend:=gmapping
```

---

## 4. 手动控制建图

如果想自己控制机器人运动：

```bash
roslaunch myrobot_description slam_mapping.launch autonomous_mapping:=false
```

另开终端：

```bash
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

## 5. 保存地图

当 RViz 中 `/map` 基本完整后，另开终端执行：

```bash
source ~/practice_ws/devel/setup.bash
roslaunch myrobot_description save_slam_map.launch
```

默认保存到：

```text
~/myrobot_description_maps/raicom_slam_map.pgm
~/myrobot_description_maps/raicom_slam_map.yaml
```

本次完整 12 点扫描并清理孤立噪点、补齐短墙线缺口后的验收地图同时保存在：

```text
~/myrobot_description_maps/raicom_slam_map_final.pgm
~/myrobot_description_maps/raicom_slam_map_final.yaml
src/myrobot_description/maps/raicom_slam_map_final.pgm
src/myrobot_description/maps/raicom_slam_map_final.yaml
```

指定保存目录和名称：

```bash
roslaunch myrobot_description save_slam_map.launch \
  output_dir:=$HOME/my_maps \
  map_name:=raicom_final_map
```

---

## 6. 关键话题

| 话题 | 类型 | 作用 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 建图使用的激光雷达 |
| `/odom` | `nav_msgs/Odometry` | 里程计 |
| `/tf` | `tf2_msgs/TFMessage` | 坐标变换 |
| `/map` | `nav_msgs/OccupancyGrid` | gmapping 输出地图 |
| `/slam_status` | `std_msgs/String` | 建图状态 JSON |
| `/task_trajectory` | `nav_msgs/Path` | 自动建图轨迹 |

---

## 7. 重要 TF 链路

SLAM 需要以下坐标关系正常：

```text
odom -> base_footprint -> base_link -> base_scan
```

检查命令：

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_link base_scan
```

如果 TF 不通，`gmapping` 无法稳定输出地图。

---

## 8. 参数文件

SLAM 参数文件：

```text
config/slam_gmapping_params.yaml
```

自动巡点路径：

```text
config/navigation_params.yaml
```

RViz 配置：

```text
config/slam.rviz
```

---

## 9. 常见问题

### 9.1 RViz 中没有地图

检查：

```bash
rostopic echo /map -n 1
rostopic echo /scan -n 1
rosnode list | grep gmapping
```

如果 `/scan` 没有数据，优先检查 Gazebo 雷达插件和 `gazebo_plugins`。

### 9.2 地图变形或重影

常见原因：机器人速度太快、转弯太急或里程计噪声导致匹配失败。可降低速度：

```text
config/task_params.yaml
```

建议调小：

```yaml
max_linear_speed: 0.15
max_lateral_speed: 0.12
max_angular_speed: 0.60
```

### 9.3 保存地图失败

先确认 `/map` 已经有数据：

```bash
rostopic echo /map -n 1
```

再执行：

```bash
roslaunch myrobot_description save_slam_map.launch
```

### 9.4 为什么加了识别图板，地图边界没变化

识别图板是 `visual-only` 物体，没有 collision。这样摄像头可以看到目标卡，但激光、代价地图和 SLAM 边界保持不变。

### 9.5 想用 SLAM 地图导航

保存地图后，启动导航时指定地图：

```bash
roslaunch myrobot_description autonomous_navigation.launch \
  map_file:=$HOME/myrobot_description_maps/raicom_slam_map.yaml
```

---

## 10. 验收点

SLAM 功能正常时，应满足：

1. `/scan` 有数据；
2. `/odom` 有数据；
3. `tf_echo odom base_footprint` 正常；
4. RViz 中能看到 `/map` 逐渐生成；
5. `save_slam_map.launch` 能生成 `.pgm` 和 `.yaml` 文件。

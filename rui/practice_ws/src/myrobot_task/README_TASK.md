# 任务演示说明

本文说明如何运行自动巡检识别任务，以及如何查看识别结果、轨迹和任务证据。

---

## 1. 任务目标

本任务用于演示机器人在仿真地图中完成：

1. 从起点出发；
2. 按路径进入识别区 1；
3. 读取识别区 1 的目标卡图片并输出敌军、友军、人质数量；
4. 继续前往识别区 2；
5. 输出识别区 2 的识别结果；
6. 返回终点；
7. 自动保存轨迹、状态和总结文件。

---

## 2. 运行方式

```bash
cd /home/chen/ros1_ultrasound_ws/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
roslaunch myrobot_task task_patrol.launch
```

如果电脑性能不足，可以关闭 RViz：

```bash
roslaunch myrobot_task task_patrol.launch launch_rviz:=false
```

当前 `task_patrol.launch` 默认使用导航栈巡航，也就是：

```text
map_server + AMCL + move_base + move_base_waypoint_navigator.py
```

如果你还需要旧版简单 `/cmd_vel` 巡点控制，使用：

```bash
roslaunch myrobot_task task_patrol_simple.launch
```

图像识别运行前提：

```bash
sudo apt install ros-$ROS_DISTRO-cv-bridge python3-opencv
```

---

## 3. 任务流程

```text
启动 Gazebo 原始世界
  ↓
加载机器人模型和仿真驱动
  ↓
map_server + AMCL + move_base
  ↓
move_base_waypoint_navigator.py 按 navigation_params.yaml 顺序发送 move_base 目标
  ↓
机器人进入识别区并停稳
  ↓
battlefield_recognition.py 读取 /camera/image_raw 并识别目标卡
  ↓
task_evidence_recorder.py 记录轨迹、状态和结果
  ↓
任务结束，生成 mission_summary.md
```

---

## 4. 关键话题

| 话题 | 类型 | 说明 |
|---|---|---|
| `/navigation_status` | `std_msgs/String` | 正式巡航开始、到达、完成、超时等状态 |
| `/recognition_result` | `std_msgs/String` | 每个识别区的识别结果 |
| `/recognition_summary` | `std_msgs/String` | 敌军、友军、人质总数 |
| `/task_trajectory` | `nav_msgs/Path` | 机器人运行轨迹 |
| `/task_markers` | `visualization_msgs/MarkerArray` | RViz 中的识别区和路径点标注 |
| `/cmd_vel` | `geometry_msgs/Twist` | 巡点控制输出 |
| `/odom` | `nav_msgs/Odometry` | 机器人里程计 |

查看命令：

```bash
rostopic echo /navigation_status
rostopic echo /recognition_result
rostopic echo /recognition_summary
```

---

## 5. 任务参数

任务参数文件：

```text
myrobot_task/config/task_params.yaml
```

### 5.1 巡点参数

正式 `task_patrol.launch` 使用
`myrobot_navigation/config/navigation_params.yaml` 中的 3 个正式导航点，
对应完整路线中的第 3、6、13 个点：`zone_1`、`right_wall_exit`、`finish`。
建图巡航路线单独维护在
`myrobot_navigation/config/slam_navigation_params.yaml`。

`myrobot_task/config/task_params.yaml` 中的 `waypoints` 只供
`task_patrol_simple.launch` 兼容使用。

字段说明：

| 字段 | 说明 |
|---|---|
| `name` | 点位名称 |
| `x`、`y` | Gazebo 世界坐标，单位 m |
| `yaw` | 机器人目标朝向，单位 rad |
| `hold` | 到达后停留时间，单位 s |

### 5.2 识别区参数

```yaml
zones:
  - name: zone_1
    center_x: 0.52
    center_y: -2.55
    radius: 0.35
    enemy: 1
    friendly: 1
    hostage: 0
```

机器人进入识别区半径范围后，会在停留时间内抓取相机画面，并输出对应识别结果。

---

## 6. 当前识别方式说明

当前默认世界已经加入两块“仅视觉、不参与碰撞”的识别面板。识别节点采用以下逻辑：

```text
机器人进入配置的识别区并停稳
  ↓
从 /camera/image_raw 获取最新画面
  ↓
YOLO ONNX 整帧检测
  ↓
宽画面重叠分块并统一 NMS
  ↓
发布识别结果与汇总结果
```

`task_params.yaml` 中的 `enemy / friendly / hostage` 现在作为验收期望值保留，识别结果来自真实图像检测，不再直接由配置生成。

### 6.1 YOLO ONNX 权重

正式权重文件位于：

```text
myrobot_recognition/recognition_weights/best.onnx
```

运行配置：

```yaml
recognition_backend: yolo_onnx
recognition_weights: recognition_weights/best.onnx
recognition_model_input_size: [640, 640]
recognition_yolo_class_names: [renzhi, youjun, dijun]
```

类别映射为 `renzhi -> hostage`、`youjun -> friendly`、
`dijun -> enemy`。正式模式会强制检查 ONNX 权重，不会静默回退模板。

---

## 7. 任务日志

运行任务后，日志默认保存在：

```text
~/.ros/myrobot_description_logs/mission_YYYYMMDD_HHMMSS/
```

包含：

```text
trajectory.csv              # 机器人轨迹
recognition_results.jsonl   # 每个识别区结果
patrol_status.jsonl         # 巡点状态
mission_summary.md          # 任务总结
```

识别截图单独保存在：

```text
~/.ros/myrobot_description_logs/recognition_frames/session_YYYYMMDD_HHMMSS/
```

查看最新总结：

```bash
rosrun myrobot_task latest_mission_summary.py
```

---

## 8. 与自主导航任务的区别

本文件默认对应的是：

```bash
roslaunch myrobot_task task_patrol.launch
```

它使用导航栈执行巡检任务。

旧版简单巡点版本是：

```bash
roslaunch myrobot_task task_patrol_simple.launch
```

它使用 `patrol_navigator.py` 直接根据 `/odom` 发布 `/cmd_vel`，属于简单巡点控制。

---

## 9. 常见问题

### 9.1 机器人不按路线走

检查：

```bash
rostopic echo /odom -n 1
rostopic echo /cmd_vel
```

如果 `/odom` 没数据，先检查仿真驱动。详见 `README_SIMULATION.md`。

### 9.2 没有识别结果或识别数量不对

检查机器人是否进入识别区：

```bash
rostopic echo /recognition_result
rostopic echo /recognition_summary
```

还需要检查相机与正式权重：

```bash
rostopic echo /camera/camera_info -n 1
rostopic echo /camera/image_raw -n 1
ls -lh $(rospack find myrobot_recognition)/recognition_weights/best.onnx
```

可以适当增大识别区半径，或调整模型置信度：

```yaml
radius: 0.50
recognition_model_confidence: 0.40
```

### 9.3 任务日志没生成

检查证据记录节点是否启动：

```bash
rosnode list | grep evidence
```

也可确认启动参数：

```bash
roslaunch myrobot_task task_patrol.launch enable_evidence_recorder:=true
```

---

## 10. 验收点

任务演示正常时，应满足：

1. Gazebo 中机器人能移动；
2. RViz 中能看到机器人、轨迹和识别区标记；
3. `/navigation_status` 持续输出状态；
4. `/recognition_result` 输出两个识别区结果；
5. `/recognition_summary` 输出累计结果；
6. 任务结束后生成 `mission_summary.md`。

# myrobot_description 项目说明

`myrobot_description` 是本工作空间的核心功能包，包含机器人模型、Gazebo 仿真、传感器插件、SLAM 建图、自主导航、任务巡检和识别结果输出。

本包适用于 ROS1 + Gazebo 仿真环境。当前版本重点完成仿真演示，不包含真实硬件的 STM32、CAN、串口或电机驱动。

---

## 1. 当前已实现功能

| 模块 | 状态 | 说明 |
|---|---:|---|
| 机器人模型 | 已完成 | 麦克纳姆轮底盘、雷达、相机、IMU 坐标系 |
| Gazebo 世界加载 | 已完成 | 使用原始 `worlds/rm_map.world` |
| 底盘仿真驱动 | 已完成 | `/cmd_vel` 控制 Gazebo 中的平面运动 |
| 轮子仿真 | 已完成 | 根据 `/cmd_vel` 逆解四轮速度并发布 `/joint_states` |
| 激光雷达 | 已完成 | 发布 `/scan` |
| 相机 | 已完成 | 发布 `/camera/image_raw` 和 `/camera/camera_info` |
| IMU | 已完成 | 发布 `/imu/data` |
| SLAM 建图 | 已完成 | `slam_gmapping` 输出 `/map` |
| 地图保存 | 已完成 | 保存 `.pgm` 和 `.yaml` |
| 自主导航 | 已完成 | `map_server + AMCL + move_base` |
| 自动巡点 | 已完成 | 默认使用 `move_base` 顺序执行导航目标点 |
| 识别结果输出 | 已完成 | 按识别区配置输出敌军、友军、人质数量 |
| 证据记录 | 已完成 | 保存轨迹、识别结果、任务状态和总结 |

---

## 2. 文件结构

```text
myrobot_description/
├── CMakeLists.txt
├── package.xml
├── README.md
├── README_SIMULATION.md
├── README_SLAM.md
├── README_NAVIGATION.md
├── README_TASK.md
├── launch/
│   ├── 1.launch                       # RViz 查看模型
│   ├── mycar_gazebo.launch            # Gazebo 基础仿真
│   ├── full_simulation.launch         # 完整仿真接口
│   ├── test_sim_drivers.launch        # 仿真驱动测试
│   ├── task_patrol.launch             # 默认巡检任务（navigate / move_base）
│   ├── task_patrol_simple.launch      # 旧版简单 /cmd_vel 巡检任务
│   ├── slam_mapping.launch            # SLAM 建图
│   ├── save_slam_map.launch           # 保存 SLAM 地图
│   ├── navigation.launch              # 导航栈
│   ├── autonomous_navigation.launch   # 一键自主导航任务
│   └── send_navigation_goals.launch   # 单独发送导航目标
├── config/
│   ├── simulation_params.yaml         # 仿真轮子驱动参数
│   ├── task_params.yaml               # 巡点和识别区参数
│   ├── slam_gmapping_params.yaml      # gmapping 参数
│   ├── navigation_params.yaml         # 自主导航目标点
│   ├── amcl_params.yaml               # AMCL 参数
│   ├── move_base_params.yaml          # move_base 参数
│   ├── costmap_common_params.yaml     # 代价地图通用参数
│   ├── global_costmap_params.yaml     # 全局代价地图参数
│   ├── local_costmap_params.yaml      # 局部代价地图参数
│   └── *.rviz                         # RViz 显示配置
├── maps/
│   ├── raicom_known_map.pgm           # 默认导航地图
│   └── raicom_known_map.yaml
├── scripts/
│   ├── mecanum_sim_driver.py          # 麦克纳姆轮状态仿真
│   ├── simulation_status_monitor.py   # 仿真话题健康监控
│   ├── patrol_navigator.py            # /cmd_vel 巡点控制
│   ├── battlefield_recognition.py     # 识别区结果输出
│   ├── task_evidence_recorder.py      # 任务证据记录
│   ├── slam_status_monitor.py         # SLAM 状态监控
│   ├── save_slam_map.py               # 保存 /map
│   ├── navigation_initializer.py      # 发布 AMCL 初始位姿
│   ├── move_base_waypoint_navigator.py# move_base 目标点发送
│   ├── navigation_status_monitor.py   # 导航健康监控
│   └── latest_mission_summary.py      # 查看最新任务总结
├── urdf/
├── meshes/
└── worlds/
```

---

## 3. 一键运行命令

### 3.1 只查看机器人模型

```bash
roslaunch myrobot_description 1.launch
```

### 3.2 启动完整仿真

```bash
roslaunch myrobot_description full_simulation.launch
```

### 3.3 自动测试仿真驱动

```bash
roslaunch myrobot_description test_sim_drivers.launch
```

### 3.4 运行巡检识别任务（默认 navigate 巡航）

```bash
roslaunch myrobot_description task_patrol.launch
```

如果你需要旧版简单 `/cmd_vel` 巡点：

```bash
roslaunch myrobot_description task_patrol_simple.launch
```

### 3.5 运行 SLAM 建图

```bash
roslaunch myrobot_description slam_mapping.launch
```

### 3.6 保存 SLAM 地图

```bash
roslaunch myrobot_description save_slam_map.launch
```

### 3.7 启动自主导航

```bash
roslaunch myrobot_description autonomous_navigation.launch
```

---

## 4. 启动文件说明

| 文件 | 用途 | 常用参数 |
|---|---|---|
| `1.launch` | RViz 查看模型 | `gui:=true/false` |
| `mycar_gazebo.launch` | Gazebo 基础仿真 | `launch_rviz:=true/false`、`enable_lidar:=true/false` |
| `full_simulation.launch` | 完整仿真接口 | `start_test_motion:=true/false` |
| `test_sim_drivers.launch` | 自动发布测试速度 | 无 |
| `task_patrol.launch` | 默认巡点识别任务（导航栈） | `map_file:=...`、`launch_rviz:=true/false` |
| `task_patrol_simple.launch` | 旧版简单巡点识别任务 | `launch_rviz:=true/false` |
| `slam_mapping.launch` | SLAM 建图 | `autonomous_mapping:=true/false` |
| `save_slam_map.launch` | 保存地图 | `output_dir:=...`、`map_name:=...` |
| `navigation.launch` | 导航栈 | `map_file:=...`、`initial_pose_x/y/a:=...` |
| `autonomous_navigation.launch` | 一键自主导航任务 | `map_file:=...`、`start_goals:=true/false` |
| `send_navigation_goals.launch` | 单独发送导航点 | `nav_params:=...` |

---

## 5. 运行结果与日志

任务日志默认写入：

```text
~/.ros/myrobot_description_logs/mission_YYYYMMDD_HHMMSS/
```

典型文件：

```text
trajectory.csv              # 机器人轨迹
recognition_results.jsonl   # 识别区结果
patrol_status.jsonl         # 巡点或导航状态
mission_summary.md          # 任务总结
```

查看最新任务总结：

```bash
rosrun myrobot_description latest_mission_summary.py
```

---

## 6. 快速自检

```bash
rosrun myrobot_description static_workspace_check.py
```

该脚本会检查 XML、launch、mesh 路径、Python 脚本权限、地图哈希等静态问题。

---

## 7. 注意事项

1. `worlds/rm_map.world` 是原始 Gazebo 场景文件，默认不需要修改。
2. 默认 `task_patrol.launch` 走导航栈，目标点优先改 `config/navigation_params.yaml`。
3. 只有在使用 `task_patrol_simple.launch` 时，才优先改 `config/task_params.yaml` 的 `waypoints`。
4. 当前底盘驱动是 Gazebo 仿真驱动，不是实体车硬件驱动。

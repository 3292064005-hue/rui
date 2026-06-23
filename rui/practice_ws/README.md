# practice_ws 使用说明

本工作空间是一个 ROS1 + Gazebo 仿真项目，按功能拆成模型、仿真、建图导航、识别和任务记录多个包。项目已配置机器人模型、Gazebo 仿真、传感器接口、SLAM 建图、自主导航、任务巡检、识别结果输出和运行证据记录。

> 说明：当前 `worlds/rm_map.world` 已加入两块仅视觉识别面板，不包含
> 碰撞几何，不改变可通行区域。路径、导航和识别参数优先在 `config/`
> 中修改。

---

## 1. 工程结构

```text
practice_ws/
├── README.md                         # 工作空间总说明
└── src/
    ├── myrobot_description/          # URDF/Xacro、材质、Gazebo 世界
    ├── myrobot_simulation/           # Gazebo 启动、底盘仿真驱动、传感器健康检查
    ├── myrobot_navigation/           # 建图、保存地图、AMCL + move_base 导航
    ├── myrobot_recognition/          # YOLO/ONNX 识别节点、权重、模板
    ├── myrobot_task/                 # 巡检任务、证据记录、任务总结
    └── my_teleop_keyboard/           # 键盘遥控节点（备用）
```

---

## 2. 环境依赖

当前工程按 Ubuntu 20.04 + ROS Noetic + Python 3.8 验证。

如果宿主机无法安装 ROS1 Noetic，推荐使用外层工作区的 Docker 环境：

```bash
cd /home/chen/ros1_ultrasound_ws
bash docker/build_noetic.sh
bash docker/run_noetic.sh
```

容器内会自动进入：

```text
/workspace/rui/rui/practice_ws
```

安装基础依赖：

```bash
sudo apt update
sudo apt install \
  ros-$ROS_DISTRO-gazebo-ros \
  ros-$ROS_DISTRO-gazebo-plugins \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-rviz \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-map-server \
  ros-$ROS_DISTRO-amcl \
  ros-$ROS_DISTRO-move-base \
  ros-$ROS_DISTRO-navigation \
  ros-$ROS_DISTRO-slam-gmapping \
  ros-$ROS_DISTRO-cv-bridge \
  python3-opencv \
  python3-numpy
```

---

## 3. 编译

```bash
cd /home/chen/ros1_ultrasound_ws/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果重新打开终端，仍需执行：

```bash
cd /home/chen/ros1_ultrasound_ws/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

---

## 4. 常用启动命令

| 目标 | 命令 |
|---|---|
| 只查看模型 | `roslaunch myrobot_simulation 1.launch` |
| 启动完整 Gazebo 仿真 | `roslaunch myrobot_simulation full_simulation.launch` |
| 仿真驱动测试（自动发布运动序列） | `roslaunch myrobot_simulation test_sim_drivers.launch` |
| 正式任务入口（推荐） | `roslaunch myrobot_task task_patrol.launch` |
| SLAM 建图 | `roslaunch myrobot_navigation slam_mapping.launch` |
| 保存 SLAM 地图 | `roslaunch myrobot_navigation save_slam_map.launch` |
| 启动导航栈，手动点目标 | `roslaunch myrobot_navigation navigation.launch` |
| 自主导航+识别+记录 | `roslaunch myrobot_navigation autonomous_navigation.launch` |
| 旧版简单巡点（兼容） | `roslaunch myrobot_task task_patrol_simple.launch` |

---

## 5. 推荐测试顺序

### 第一步：确认模型能显示

```bash
roslaunch myrobot_simulation 1.launch
```

RViz 中应能看到机器人模型、轮子、雷达和相机坐标系。

### 第二步：确认仿真驱动正常

```bash
roslaunch myrobot_simulation test_sim_drivers.launch
```

检查话题：

```bash
rostopic echo /odom -n 1
rostopic echo /joint_states -n 1
rostopic echo /scan -n 1
rostopic echo /imu/data -n 1
rostopic echo /camera/image_raw -n 1
rostopic echo /camera/camera_info -n 1
rostopic echo /simulation_health
```

### 第三步：运行 SLAM 建图

```bash
roslaunch myrobot_navigation slam_mapping.launch
```

建图完成后保存：

```bash
roslaunch myrobot_navigation save_slam_map.launch
```

### 第四步：运行自主导航

```bash
roslaunch myrobot_navigation autonomous_navigation.launch
```

查看状态：

```bash
rostopic echo /navigation_status
rostopic echo /navigation_health
rostopic echo /move_base/status
rostopic echo /recognition_result
rostopic echo /recognition_summary
```

### 第五步：查看任务结果

```bash
rosrun myrobot_task latest_mission_summary.py
```

如需键盘手动遥控：

```bash
rosrun my_teleop_keyboard teleop_node.py
```

任务日志默认保存在：

```text
~/.ros/myrobot_description_logs/
```

---

## 6. 主要话题

| 话题 | 作用 |
|---|---|
| `/cmd_vel` | 底盘速度控制输入，支持前后、横移、旋转 |
| `/odom` | 里程计 |
| `/joint_states` | 轮子关节状态 |
| `/mecanum_wheel_speeds` | 四个麦克纳姆轮角速度 |
| `/scan` | 激光雷达原始数据 |
| `/scan_filtered` | 自车近场过滤后的激光（建图/导航使用） |
| `/camera/image_raw` | 相机图像 |
| `/camera/camera_info` | 相机内参信息 |
| `/imu/data` | IMU 数据 |
| `/map` | SLAM 或 map_server 输出地图 |
| `/amcl_pose` | AMCL 定位结果 |
| `/move_base/status` | move_base action 状态 |
| `/navigation_status` | 自主导航任务状态 |
| `/navigation_health` | 导航链路健康状态 |
| `/recognition_result` | 单个识别区结果 |
| `/recognition_summary` | 识别结果汇总 |
| `/task_trajectory` | 任务轨迹 |
| `/simulation_health` | 仿真接口健康状态 |

---

## 7. 修改参数的位置

| 要修改的内容 | 文件 |
|---|---|
| 正式巡航目标点 | `src/myrobot_navigation/config/navigation_params.yaml` |
| 建图巡航目标点 | `src/myrobot_navigation/config/slam_navigation_params.yaml` |
| 识别区、识别模型参数 | `src/myrobot_task/config/task_params.yaml` |
| SLAM 参数 | `src/myrobot_navigation/launch/slam_mapping.launch`，对比后端参数见 `src/myrobot_navigation/config/slam_*.yaml` |
| 兵人识别权重 | `src/myrobot_recognition/recognition_weights/` |
| AMCL 参数 | `src/myrobot_navigation/config/amcl_params.yaml` |
| move_base 参数 | `src/myrobot_navigation/config/move_base_params.yaml` |
| TEB 局部规划参数 | `src/myrobot_navigation/config/base_local_planner_params.yaml` |
| 代价地图参数 | `src/myrobot_navigation/config/*costmap*_params.yaml` |
| 仿真轮子参数 | `src/myrobot_simulation/config/simulation_params.yaml` |
| 机器人模型 | `src/myrobot_description/urdf/turtlebot3_mecanum.urdf.xacro` |

---

## 8. 说明文档索引

建议按以下顺序阅读。参数数值以 `config/*.yaml` 和 `launch/*.launch`
为最终依据，文档中的片段仅用于说明：

1. `src/myrobot_description/README.md`：项目总览与资源索引；
2. `src/myrobot_simulation/README_DETAILS.md`：仿真驱动和传感器；
3. `src/myrobot_navigation/README_SLAM.md`：建图流程；
4. `src/myrobot_navigation/README_NAVIGATION.md`：自主导航流程；
5. `src/myrobot_task/README_TASK.md`：任务演示和结果记录；
6. `REPORT.md`：比赛技术报告、算法说明与验收数据。

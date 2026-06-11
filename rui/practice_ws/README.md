# practice_ws 使用说明

本工作空间是一个 ROS1 + Gazebo 仿真项目，核心包为 `myrobot_description`。项目已配置机器人模型、Gazebo 仿真、传感器接口、SLAM 建图、自主导航、任务巡检、识别结果输出和运行证据记录。

> 说明：当前 `worlds/rm_map.world` 已加入两块仅视觉识别面板，不包含
> 碰撞几何，不改变可通行区域。路径、导航和识别参数优先在 `config/`
> 中修改。

---

## 1. 工程结构

```text
practice_ws/
├── README.md                         # 工作空间总说明
└── src/
    └── myrobot_description/
        ├── README.md                 # 包级总说明
        ├── README_SIMULATION.md      # Gazebo 仿真接口说明
        ├── README_SLAM.md            # slam_toolbox 建图与对比后端说明
        ├── README_NAVIGATION.md      # AMCL + move_base 自主导航说明
        ├── README_TASK.md            # 巡检识别任务说明
        ├── launch/                   # 启动文件
        ├── config/                   # 参数和 RViz 配置
        ├── maps/                     # 导航用静态地图
        ├── scripts/                  # Python 节点
        ├── urdf/                     # 机器人模型
        ├── meshes/                   # STL 模型文件
        └── worlds/                   # Gazebo 世界文件
```

---

## 2. 环境依赖

当前工程按 Ubuntu 20.04 + ROS Noetic + Python 3.8 验证。

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
  ros-$ROS_DISTRO-slam-toolbox \
  ros-$ROS_DISTRO-slam-gmapping \
  ros-$ROS_DISTRO-cv-bridge \
  python3-opencv \
  python3-numpy
```

---

## 3. 编译

```bash
cd /root/workspace/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

如果重新打开终端，仍需执行：

```bash
cd /root/workspace/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

---

## 4. 常用启动命令

| 目标 | 命令 |
|---|---|
| 只查看模型 | `roslaunch myrobot_description 1.launch` |
| 启动完整 Gazebo 仿真 | `roslaunch myrobot_description full_simulation.launch` |
| 测试底盘、轮子、雷达、相机、IMU | `roslaunch myrobot_description test_sim_drivers.launch` |
| 自动巡点与识别演示 | `roslaunch myrobot_description task_patrol.launch` |
| SLAM 建图 | `roslaunch myrobot_description slam_mapping.launch` |
| 保存 SLAM 地图 | `roslaunch myrobot_description save_slam_map.launch` |
| 启动导航栈，手动点目标 | `roslaunch myrobot_description navigation.launch` |
| 一键自主导航任务 | `roslaunch myrobot_description autonomous_navigation.launch` |

---

## 5. 推荐测试顺序

### 第一步：确认模型能显示

```bash
roslaunch myrobot_description 1.launch
```

RViz 中应能看到机器人模型、轮子、雷达和相机坐标系。

### 第二步：确认仿真驱动正常

```bash
roslaunch myrobot_description test_sim_drivers.launch
```

检查话题：

```bash
rostopic echo /odom -n 1
rostopic echo /joint_states -n 1
rostopic echo /scan -n 1
rostopic echo /imu/data -n 1
rostopic echo /camera/camera_info -n 1
rostopic echo /simulation_health
```

### 第三步：运行 SLAM 建图

```bash
roslaunch myrobot_description slam_mapping.launch
```

建图完成后保存：

```bash
roslaunch myrobot_description save_slam_map.launch
```

### 第四步：运行自主导航

```bash
roslaunch myrobot_description autonomous_navigation.launch
```

查看状态：

```bash
rostopic echo /navigation_status
rostopic echo /navigation_health
rostopic echo /move_base/status
```

### 第五步：查看任务结果

```bash
rosrun myrobot_description latest_mission_summary.py
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
| `/scan` | 激光雷达数据 |
| `/camera/image_raw` | 相机图像 |
| `/camera/camera_info` | 相机内参信息 |
| `/imu/data` | IMU 数据 |
| `/map` | SLAM 或 map_server 输出地图 |
| `/amcl_pose` | AMCL 定位结果 |
| `/move_base/status` | move_base action 状态 |
| `/navigation_status` | 自主导航任务状态 |
| `/recognition_result` | 单个识别区结果 |
| `/recognition_summary` | 识别结果汇总 |
| `/task_trajectory` | 任务轨迹 |
| `/simulation_health` | 仿真接口健康状态 |

---

## 7. 修改参数的位置

| 要修改的内容 | 文件 |
|---|---|
| 正式巡航目标点 | `src/myrobot_description/config/navigation_params.yaml` |
| 建图巡航目标点 | `src/myrobot_description/config/slam_navigation_params.yaml` |
| 识别区、识别模型参数 | `src/myrobot_description/config/task_params.yaml` |
| SLAM 参数 | `src/myrobot_description/config/slam_toolbox_params.yaml`、`src/myrobot_description/launch/slam_mapping.launch` |
| 兵人识别权重 | `src/myrobot_description/recognition_weights/` |
| AMCL 参数 | `src/myrobot_description/config/amcl_params.yaml` |
| move_base 参数 | `src/myrobot_description/config/move_base_params.yaml` |
| 代价地图参数 | `src/myrobot_description/config/*costmap*_params.yaml` |
| 仿真轮子参数 | `src/myrobot_description/config/simulation_params.yaml` |
| 机器人模型 | `src/myrobot_description/urdf/turtlebot3_mecanum.urdf.xacro` |

---

## 8. 说明文档索引

建议按以下顺序阅读。参数数值以 `config/*.yaml` 和 `launch/*.launch`
为最终依据，文档中的片段仅用于说明：

1. `src/myrobot_description/README.md`：项目总说明；
2. `src/myrobot_description/README_SIMULATION.md`：仿真驱动和传感器；
3. `src/myrobot_description/README_SLAM.md`：建图流程；
4. `src/myrobot_description/README_NAVIGATION.md`：自主导航流程；
5. `src/myrobot_description/README_TASK.md`：任务演示和结果记录；
6. `REPORT.md`：比赛技术报告、算法说明与验收数据。

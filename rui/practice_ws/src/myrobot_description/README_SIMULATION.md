# Gazebo 仿真说明

本文说明本包的 Gazebo 仿真接口，包括底盘、轮子、里程计、雷达、相机、IMU 和健康状态监控。

---

## 1. 仿真目标

当前仿真层提供完整的机器人运行接口：

```text
/cmd_vel
  ├── 控制 Gazebo 中机器人运动
  └── 驱动四个麦克纳姆轮的 joint_states 动画

Gazebo 传感器
  ├── /scan
  ├── /camera/image_raw
  ├── /camera/camera_info
  └── /imu/data
```

其中：

- 机器人在 Gazebo 中的真实位姿由 `libgazebo_ros_planar_move.so` 驱动；
- 四个轮子的转动由 `scripts/mecanum_sim_driver.py` 根据 `/cmd_vel` 计算并发布；
- 雷达、相机、IMU 由 URDF/Xacro 中的 Gazebo 插件发布。

---

## 2. 一键启动完整仿真

```bash
cd ~/practice_ws
catkin_make
source devel/setup.bash
roslaunch myrobot_description full_simulation.launch
```

默认会启动：

1. Gazebo 和带视觉识别面板的 `rm_map.world`；
2. 机器人模型；
3. 底盘平面运动插件；
4. 轮子状态仿真节点；
5. 雷达、相机、IMU；
6. RViz；
7. 仿真健康监控节点。

如果不需要 RViz：

```bash
roslaunch myrobot_description full_simulation.launch launch_rviz:=false
```

---

## 3. 自动测试仿真驱动

```bash
roslaunch myrobot_description test_sim_drivers.launch
```

该启动文件会自动发布一段 `/cmd_vel`，用于检查机器人是否能完成：

- 前进；
- 横移；
- 后退；
- 原地旋转；
- 轮子随速度指令转动；
- 雷达、相机、IMU 正常输出。

也可以直接使用：

```bash
roslaunch myrobot_description full_simulation.launch start_test_motion:=true
```

---

## 4. 关键话题

| 模块 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 控制输入 | `/cmd_vel` | `geometry_msgs/Twist` | 底盘速度输入，支持 x、y、yaw |
| 里程计 | `/odom` | `nav_msgs/Odometry` | Gazebo 底盘插件发布 |
| 轮子关节 | `/joint_states` | `sensor_msgs/JointState` | 四个轮子的角度和角速度 |
| 轮速 | `/mecanum_wheel_speeds` | `std_msgs/Float64MultiArray` | 顺序为 LF、RF、LR、RR |
| 雷达 | `/scan` | `sensor_msgs/LaserScan` | 360° 激光雷达 |
| 相机图像 | `/camera/image_raw` | `sensor_msgs/Image` | RGB 图像 |
| 相机参数 | `/camera/camera_info` | `sensor_msgs/CameraInfo` | 相机内参 |
| IMU | `/imu/data` | `sensor_msgs/Imu` | 惯性测量数据 |
| 健康状态 | `/simulation_health` | `std_msgs/String` | 仿真接口是否在线 |

---

## 5. 手动检查命令

```bash
rostopic echo /odom -n 1
rostopic echo /joint_states -n 1
rostopic echo /mecanum_wheel_speeds -n 1
rostopic echo /scan -n 1
rostopic echo /imu/data -n 1
rostopic echo /camera/camera_info -n 1
rostopic echo /simulation_health
```

检查 TF：

```bash
rosrun tf tf_echo odom base_footprint
rosrun tf tf_echo base_link base_scan
rosrun tf tf_echo base_link camera_link
rosrun tf tf_echo base_link imu_link
```

---

## 6. 底盘和轮子仿真原理

### 6.1 Gazebo 平面运动

Gazebo 中实际驱动机器人移动的是：

```text
libgazebo_ros_planar_move.so
```

它负责：

- 订阅 `/cmd_vel`；
- 在 Gazebo 中更新机器人位姿；
- 发布 `/odom`；
- 发布 `odom -> base_footprint` TF。

### 6.2 麦克纳姆轮状态仿真

`scripts/mecanum_sim_driver.py` 使用麦克纳姆轮逆运动学，把同一个 `/cmd_vel` 转成四个轮子的角速度：

```text
linear.x   -> 前后运动
linear.y   -> 左右横移
angular.z  -> 原地旋转
```

输出：

```text
/joint_states
/mecanum_wheel_speeds
```

这使 RViz 中四个轮子能按速度指令转动，也方便展示“轮子驱动逻辑”。

---

## 7. 参数文件

仿真轮子参数位于：

```text
config/simulation_params.yaml
```

常用字段：

```yaml
wheel_radius: 0.033
half_wheel_base: 0.0638
half_track_width: 0.0850
joint_state_rate: 50.0
cmd_vel_timeout: 0.5
max_wheel_angular_speed: 45.0
```

一般不需要修改。如果轮子转动速度看起来过快或过慢，可以优先调整 `wheel_radius` 或 `max_wheel_angular_speed`。

---

## 8. 常见问题

### 8.1 机器人不动

检查 `/cmd_vel` 和 `/odom`：

```bash
rostopic echo /cmd_vel
rostopic echo /odom -n 1
```

如果 `/cmd_vel` 有数据但 `/odom` 没数据，检查 Gazebo 插件依赖：

```bash
sudo apt install ros-$ROS_DISTRO-gazebo-plugins
```

### 8.2 轮子不转

检查：

```bash
rostopic echo /joint_states -n 1
rosnode list | grep mecanum
```

如果没有 `mecanum_sim_driver`，确认启动文件中 `start_wheel_joint_driver:=true`。

### 8.3 没有雷达数据

```bash
rostopic echo /scan -n 1
```

如果没有数据，检查 `enable_lidar:=true`，并确认 `gazebo_plugins` 已安装。

### 8.4 没有相机或 IMU 数据

```bash
rostopic echo /camera/camera_info -n 1
rostopic echo /imu/data -n 1
```

检查启动参数：

```text
enable_camera:=true
enable_imu:=true
```

---

## 9. 适用范围

当前为仿真驱动，适合：

- Gazebo 演示；
- SLAM 建图；
- AMCL 定位；
- move_base 自主导航；
- 任务路线和识别结果展示。

不包含：

- 实体车电机驱动；
- 串口/CAN 通信；
- 编码器硬件反馈；
- 真实 IMU 或真实雷达驱动。

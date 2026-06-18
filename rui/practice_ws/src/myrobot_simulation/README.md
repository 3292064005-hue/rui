# myrobot_simulation

Gazebo 仿真包，负责启动世界、加载机器人模型、发布仿真底盘和传感器数据。

## 入口

```bash
roslaunch myrobot_simulation full_simulation.launch
roslaunch myrobot_simulation mycar_gazebo.launch
roslaunch myrobot_simulation test_sim_drivers.launch
roslaunch myrobot_simulation 1.launch
```

## 内容

- `launch/`：Gazebo、RViz、模型显示、驱动测试入口
- `config/simulation_params.yaml`：麦克纳姆轮仿真参数
- `scripts/mecanum_sim_driver.py`：`/cmd_vel` 到轮速和 `/joint_states`
- `scripts/simulation_status_monitor.py`：仿真话题健康检查
- `scripts/cmd_vel_test_motion.py`：测试速度序列

## 关键话题

- `/cmd_vel`
- `/odom`
- `/joint_states`
- `/scan`
- `/camera/image_raw`
- `/imu/data`

机器人描述、mesh、world 和材质来自 `myrobot_description`。

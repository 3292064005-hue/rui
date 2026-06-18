# myrobot_navigation

导航和建图包，负责 SLAM、地图保存、AMCL、move_base 和自动目标点发送。

## 入口

```bash
roslaunch myrobot_navigation navigation.launch
roslaunch myrobot_navigation autonomous_navigation.launch
roslaunch myrobot_navigation slam_mapping.launch
roslaunch myrobot_navigation save_slam_map.launch
roslaunch myrobot_navigation send_navigation_goals.launch
```

## 内容

- `launch/navigation.launch`：Gazebo + map_server + AMCL + move_base
- `launch/autonomous_navigation.launch`：导航、识别、记录、目标点发送的一体入口
- `launch/slam_mapping.launch`：自动建图入口
- `config/navigation_params.yaml`：正式巡航目标和内部过渡点
- `config/slam_navigation_params.yaml`：建图巡航路线
- `config/base_local_planner_params.yaml`：TEB 全向底盘局部规划参数
- `maps/`：map_server 使用的地图文件
- `scripts/move_base_waypoint_navigator.py`：顺序发送导航目标

## 默认路线

正式任务目标为：

```text
zone_1 -> right_wall_exit -> zone_2 -> finish
```

任务入口通常使用：

```bash
roslaunch myrobot_task task_patrol.launch
```

该入口会调用本包的 `autonomous_navigation.launch`。

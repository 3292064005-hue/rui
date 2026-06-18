# myrobot_task

任务编排包，负责比赛巡检入口、证据记录、最新总结查看和静态检查。

## 推荐入口

```bash
roslaunch myrobot_task task_patrol.launch
```

该入口会启动：

- `myrobot_simulation`：Gazebo 和机器人仿真
- `myrobot_navigation`：map_server、AMCL、move_base、目标点发送
- `myrobot_recognition`：识别区图像识别
- `myrobot_task`：轨迹、状态、识别结果记录

## 其他入口

```bash
roslaunch myrobot_task task_patrol_simple.launch
rosrun myrobot_task latest_mission_summary.py
rosrun myrobot_task static_workspace_check.py
```

`task_patrol_simple.launch` 是旧版 `/cmd_vel` 巡点控制，仅用于兼容。

## 参数

- `config/task_params.yaml`：识别区、识别模型、日志和旧版简单巡点参数
- `config/task.rviz`：任务 RViz 显示配置

正式导航路线不在本包维护，而在：

```text
myrobot_navigation/config/navigation_params.yaml
```

## 日志

默认输出到：

```text
~/.ros/myrobot_description_logs/
```

常见文件：

- `trajectory.csv`
- `recognition_results.jsonl`
- `patrol_status.jsonl`
- `mission_summary.md`

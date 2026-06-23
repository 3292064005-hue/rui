# myrobot 项目总览

这是一个按功能拆分的 ROS1 + Gazebo 工作区。

## 包分工

- `myrobot_description`：URDF、mesh、world、材质资源
- `myrobot_simulation`：Gazebo 仿真和底盘驱动
- `myrobot_navigation`：SLAM、地图、导航、目标点发送
- `myrobot_recognition`：图像识别、模板、权重、运行时
- `myrobot_task`：任务入口、日志、汇总、静态检查

## 推荐入口

```bash
roslaunch myrobot_task task_patrol.launch
```

它会串起仿真、导航、识别和任务记录，是最优默认入口。

## 常用命令

```bash
roslaunch myrobot_simulation full_simulation.launch
roslaunch myrobot_navigation navigation.launch
roslaunch myrobot_navigation slam_mapping.launch
roslaunch myrobot_navigation save_slam_map.launch
roslaunch myrobot_task task_patrol_simple.launch
rosrun myrobot_task latest_mission_summary.py
rosrun myrobot_task static_workspace_check.py
```

## 资源路径

- 机器人模型：`myrobot_description/urdf/`
- 仿真参数：`myrobot_simulation/config/`
- 导航参数和地图：`myrobot_navigation/config/`、`myrobot_navigation/maps/`
- 识别权重和模板：`myrobot_recognition/recognition_weights/`、`myrobot_recognition/recognition_templates/`
- 任务参数：`myrobot_task/config/task_params.yaml`

## 说明文档

- [仿真说明](../myrobot_simulation/README_DETAILS.md)
- [SLAM 说明](../myrobot_navigation/README_SLAM.md)
- [导航说明](../myrobot_navigation/README_NAVIGATION.md)
- [任务说明](../myrobot_task/README_TASK.md)
- [比赛符合性说明](../myrobot_task/README_COMPETITION_ALIGNMENT.md)
- [验收清单](ACCEPTANCE_CHECKLIST.md)

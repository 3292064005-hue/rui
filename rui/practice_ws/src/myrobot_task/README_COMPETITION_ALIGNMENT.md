# 比赛符合性说明

本文说明当前仓库如何对应 RAICOM 智能侦察仿真任务的关键要求。

## 1. 当前实现

- 地图仍使用 5 m x 4 m 默认比赛场地。
- `rm_map.world` 中已加入两块视觉识别面板：
  - `zone_1` 面板内容为 `1 敌军 + 1 友军`
  - `zone_2` 面板内容为 `2 敌军 + 1 人质`
- 识别节点使用 `/camera/image_raw` 和正式 YOLO ONNX 检测权重。
- SLAM 默认使用 `slam_toolbox`，基于 `/scan_filtered + /odom + TF` 做稳定高分辨率建图。
- 自主导航仍使用 `map_server + AMCL + move_base`。

## 2. 为什么识别面板不影响建图和导航

- 识别面板由贴图前表面和不透明背板组成，只添加 `visual`，没有
  `collision`。
- 激光雷达不会把这些图板当作障碍物。
- 最终 SLAM 地图、`scan_filtered` 和 `move_base` 参数可以继续使用。

## 3. 识别结果来源

识别结果不再由配置文件中的计数直接生成，而是按以下流程输出：

```text
进入识别区
  ↓
等待短时停稳
  ↓
读取相机画面
  ↓
YOLO ONNX 整帧检测
  ↓
宽画面重叠分块并统一 NMS
  ↓
输出敌军 / 友军 / 人质数量
```

`myrobot_task/config/task_params.yaml` 中保留的 `enemy / friendly / hostage` 字段，仅用于验收对照和日志核查。

正式权重为 `myrobot_recognition/recognition_weights/best.onnx`，运行后端为
`yolo_onnx`。模型原始类别映射为 `renzhi -> hostage`、
`youjun -> friendly`、`dijun -> enemy`；识别话题保持不变。

## 4. 验收建议

- 巡检演示：`roslaunch myrobot_task task_patrol.launch`
- 自主导航演示：`roslaunch myrobot_navigation autonomous_navigation.launch`
- 建图演示：`roslaunch myrobot_navigation slam_mapping.launch`
- 保存地图：`roslaunch myrobot_navigation save_slam_map.launch`

识别完成后，截图默认保存在：

```text
~/.ros/myrobot_description_logs/recognition_frames/
```

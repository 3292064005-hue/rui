# 比赛符合性说明

本文说明当前仓库如何对应 RAICOM 智能侦察仿真任务的关键要求。

## 1. 当前实现

- 地图仍使用 5 m x 4 m 默认比赛场地。
- `rm_map.world` 中已加入两块视觉识别面板：
  - `zone_1` 面板内容为 `1 敌军 + 1 友军`
  - `zone_2` 面板内容为 `2 敌军 + 1 人质`
- 识别节点使用 `/camera/image_raw`，支持模板匹配和可选 ONNX 权重分类。
- SLAM 默认基于 `/scan + /odom + TF` 的固定边界栅格建图。
- 自主导航仍使用 `map_server + AMCL + move_base`。

## 2. 为什么识别面板不影响建图和导航

- 识别面板只添加了 `visual`，没有添加 `collision`。
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
提取白底目标卡
  ↓
ORB 模板匹配
  ↓
必要时回退灰度模板匹配
  ↓
输出敌军 / 友军 / 人质数量
```

`config/task_params.yaml` 中保留的 `enemy / friendly / hostage` 字段，仅用于验收对照和日志核查。

正式兵人权重可放入 `recognition_weights/`，并通过
`recognition_weights` 参数启用；权重接口不改变现有识别话题。

## 4. 验收建议

- 巡检演示：`roslaunch myrobot_description task_patrol.launch`
- 自主导航演示：`roslaunch myrobot_description autonomous_navigation.launch`
- 建图演示：`roslaunch myrobot_description slam_mapping.launch`
- 保存地图：`roslaunch myrobot_description save_slam_map.launch`

识别完成后，截图默认保存在：

```text
~/.ros/myrobot_description_logs/recognition_frames/
```

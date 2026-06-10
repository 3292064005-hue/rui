# 项目验收清单

## 构建与静态检查

```bash
cd /root/workspace/rui/rui/practice_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
python3 -m py_compile src/myrobot_description/scripts/*.py
rosrun myrobot_description static_workspace_check.py
```

预期：构建成功，静态检查输出 `Static check passed.`。

## SLAM

```bash
roslaunch myrobot_description slam_mapping.launch
```

验收：

- `/map/info` 为 `295 × 245`、约 `0.02m`；
- 自动巡航读取 `navigation_params.yaml`；
- 默认完整巡航 3 圈；
- 地图无旋转、重影和场外长射线。

最终地图：

```text
maps/raicom_slam_map_final.pgm
maps/raicom_slam_map_final.yaml
```

## 导航

```bash
roslaunch myrobot_description autonomous_navigation.launch
```

验收：

- `map_server`、`amcl`、`move_base` 存活；
- `/amcl_pose` 有效；
- 平移时不旋转，到点后原地调整方向；
- `/navigation_status` 最终输出 `patrol_completed`。

## 图像识别

```bash
roslaunch myrobot_description task_patrol.launch
```

验收：

- `/camera/image_raw` 有数据；
- `/recognition_result` 包含检测明细和证据图片路径；
- `/recognition_summary` 汇总两个区域；
- 面板为不透明视觉物体，后墙不会从面板区域透出；
- `recognition_backend` 为 `yolo_onnx`；
- 区域 1 输出 `1 敌军 + 1 友军`；
- 区域 2 输出 `2 敌军 + 1 人质`。

## 权重接口

权重路径：

```text
recognition_weights/best.pt
recognition_weights/best.onnx
```

参数：

```yaml
recognition_backend: yolo_onnx
recognition_weights: recognition_weights/best.onnx
recognition_model_input_size: [640, 640]
recognition_yolo_class_names: [renzhi, youjun, dijun]
```

## 任务证据

确认以下文件生成：

```text
trajectory.csv
recognition_results.jsonl
patrol_status.jsonl
mission_summary.md
recognition_frames/*_raw.jpg
recognition_frames/*_annotated.jpg
```

类别映射：`renzhi -> hostage`、`youjun -> friendly`、
`dijun -> enemy`。

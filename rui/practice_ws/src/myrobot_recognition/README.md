# myrobot_recognition

图像识别包，负责识别区的敌军、友军、人质计数。

## 内容

- `scripts/battlefield_recognition.py`：识别节点
- `recognition_weights/best.onnx`：YOLO ONNX 权重
- `recognition_weights/best.pt`：训练权重备份
- `recognition_templates/`：模板识别兼容资源
- `python_vendor/`：Python 3.8 CPU ONNX Runtime

## 运行方式

通常不单独启动，由任务入口加载：

```bash
roslaunch myrobot_task task_patrol.launch
```

识别节点读取：

- `/camera/image_raw`
- `/odom`
- `/navigation_status` 或 `/patrol_status`

输出：

- `/recognition_result`
- `/recognition_summary`

## 参数来源

识别参数位于：

```text
myrobot_task/config/task_params.yaml
```

权重参数保持包内相对路径：

```yaml
recognition_backend: yolo_onnx
recognition_weights: recognition_weights/best.onnx
```

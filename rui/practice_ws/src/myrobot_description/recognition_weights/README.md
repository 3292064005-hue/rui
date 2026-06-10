# 兵人图像识别权重

正式识别使用从 `best.pt` 导出的 YOLO ONNX 检测模型。ROS 功能包自带
CPU 版 ONNX Runtime，不依赖运行时安装 PyTorch、Ultralytics 或新版
OpenCV。

模型文件：

```text
best.pt
best.onnx
```

当前训练类别到任务类别的映射：

```text
renzhi -> hostage
youjun -> friendly
dijun  -> enemy
```

运行配置：

```yaml
recognition_backend: yolo_onnx
recognition_weights: recognition_weights/best.onnx
recognition_model_input_size: [640, 640]
recognition_model_confidence: 0.50
recognition_yolo_iou_threshold: 0.45
recognition_yolo_class_names: [renzhi, youjun, dijun]
recognition_yolo_label_map:
  renzhi: hostage
  youjun: friendly
  dijun: enemy
```

`yolo_onnx` 会在整张裁剪后的相机画面中检测目标，并对同类检测执行 NMS。
权重缺失、格式错误或 OpenCV 无法加载时，节点会直接终止，避免正式任务
悄悄退回模板识别。

# 兵人图像识别权重接口

此目录预留兵人分类模型权重。当前支持 OpenCV DNN 可读取的 ONNX
分类模型，模型输入为单张已完成透视校正的目标卡图片，输出为类别 logits。

推荐文件名：

```text
soldier_classifier.onnx
```

启用方式：

```yaml
recognition_backend: auto
recognition_weights: recognition_weights/soldier_classifier.onnx
recognition_model_labels: [enemy, friendly, hostage]
recognition_model_input_size: [224, 224]
recognition_model_confidence: 0.60
```

`auto` 模式下，权重不存在或加载失败时自动使用现有 ORB/模板识别。
`onnx` 模式下，权重缺失或格式错误会直接终止节点，适合正式验收。

模型约定：

- 格式：ONNX
- 输入：`NCHW`，三通道图片
- 输出：一个长度与 `recognition_model_labels` 相同的 logits 向量
- 标签顺序必须与训练时类别索引完全一致
- 当前接口用于分类；白底目标卡定位仍由 OpenCV 轮廓算法完成

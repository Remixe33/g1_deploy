# 相机模块

相机模块描述设备流格式、视场、安装外参和处理管线。任务档案只引用模块，不复制这些参数。

## 当前模块

- `d435_rgbd.json`：D435 深度和独立 RGB，默认 640×480@30，供 door 原生感知。
- `d435_depth_480x270_60hz.json`：D435 480×270@60，供 Parkour 策略预处理管线。

## 常用字段

- `stream`：实机深度分辨率与帧率。
- `simulation_stream`：仿真深度分辨率。
- `pipeline`：`none` 仅 Raw；`parkour` 生成 Raw 和 Model。
- `scene_camera`：MuJoCo 场景相机名。
- `horizontal_fov_deg` / `vertical_fov_deg`：默认画幅的视场记录；仿真水平视场会按实际画幅重新计算。
- `position_in_torso_m`：深度光学原点在 torso 中的位置。
- `orientation_wxyz_in_torso_forward_left_up`：深度安装朝向。
- `supported_streams`：官方标称格式和帧率。
- `rgb`：可选独立 RGB 配置。

## 官方支持格式

数据来自 Intel D400 系列数据手册 Table 4-2（USB 3.1 Gen1）。

深度 Z16：

```text
1280×720：6/15/30 FPS
848×480、640×480、640×360、480×270、424×240：6/15/30/60/90 FPS
```

RGB YUY2：

```text
1920×1080、1280×720：6/15/30 FPS
960×540、848×480、640×480、640×360、424×240：6/15/30/60 FPS
320×240、320×180：6/30/60 FPS
```

选择其他格式时修改对应 `stream`。仿真按 `2*atan(tan(VFOV/2)*width/height)` 计算水平视场；真实设备应使用对应流的实际内参。

## RGB 与深度外参

D435 深度坐标原点与左红外成像器重合，RGB 来自独立传感器，不能共用同一仿真镜头。

`rgb` 节包含：

- `scene_camera` 和 `stream`；
- RGB 独立视场；
- `position_in_depth_optical_m`：深度光学坐标系中的平移，x 右、y 下、z 前，单位米；
- `orientation_wxyz_in_depth_optical`：RGB 到深度的旋转四元数；
- `calibration_source`：参数来源及是否实测。

当前 15 mm 水平间距来自 Intel D400 数据手册 Figure 10-9，旋转为近似值，不是设备实测标定。部署前使用：

```python
extrinsics = depth_profile.get_extrinsics_to(color_profile)
intrinsics = color_profile.get_intrinsics()
```

设备实际流格式可通过 `get_stream_profiles()` 或 `rs-enumerate-devices` 枚举。高级模式 JSON 视觉预设属于成像/后处理配置，不等同于内参或外参标定。

实机单个 D435 深度传感器不能同时提供 Parkour 策略管线和另一路原生深度流；相关档案组合会被拒绝。

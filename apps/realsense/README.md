# RealSense 采集与独立预览

本目录包含 C++ 原生深度采集库、只读相机检查和独立 Python 网页预览。Parkour/Hybrid 实机控制器直接使用 C++ 库，不依赖网页服务。

## 原生相机检查

```bash
./build/g1_camera_check
./build/g1_camera_check --camera-config profiles/configs/cameras/d435_rgbd.json
```

程序只采集并验证 60 张新鲜深度帧，不初始化 DDS、不发送机器人命令。D435 door 配置默认 640×480@30，其他格式见 [相机模块说明](../../profiles/configs/cameras/README.md)。

构建自动检测 `librealsense2` C++ SDK；机器人构建建议使用：

```bash
cmake -S . -B build -DG1_REQUIRE_REALSENSE=ON \
  -DONNXRUNTIME_ROOT=/path/to/native/onnxruntime
```

仅安装 `pyrealsense2` 不提供 C++ 链接依赖。SDK 位于私有目录时使用 `CMAKE_PREFIX_PATH` 或 `realsense2_DIR`。

## 独立网页预览

```bash
./g1 realsense --host 0.0.0.0
./g1 realsense --host 0.0.0.0 --color
./g1 realsense --mock
```

在可信网络中用 `--host 0.0.0.0` 启动后，从工控机打开日志中可达的 PC2 地址。G1 默认地址是 [http://192.168.123.164:8766/](http://192.168.123.164:8766/)，`0.0.0.0` 只是监听地址。不带 `--host` 时仅限机载电脑本机访问。

默认显示原始深度和 32×18 策略深度，`--color` 额外显示 RGB，`--mock` 只生成合成网页画面，不供策略使用。

- 在反馈机上，原厂图传占用了色彩节点，显式开启 RGB 还曾触发原生开流段错误。先用深度自检；只有在相机空闲时才分别测试仅 RGB 和双流，项目不会停止原厂图传。
- Python 依赖见 `requirements.txt`，可用 `PYTHON_BIN` 指定解释器。
- `--serial` 选择设备，`--width/--height/--fps` 选择深度模式。
- `--camera-mode legacy` 保留旧处理方式。
- 默认仅监听 `127.0.0.1`；`--token` 可开启 HTTP Basic，用户名为 `camera`。
- 启动实机控制前必须退出网页，避免重复占用 USB。

## 实机数据链路

```text
RealSense USB
  → C++ 采集线程
  → Z16 × depth scale 转米制 float32
  → 进程内最新帧缓存
  → 50 Hz 策略线程预处理与推理
```

不使用 Python、UDP、HTTP 或跨进程共享内存传递实机策略深度。设备时钟映射到主机单调时钟，延迟帧不会重新打时间戳；断流、重启和采集异常会传递给控制器。

策略后处理与推理按 20 ms 周期调度，控制线程按 2 ms 调度。网页使用 HTTP/MJPEG，仅供控制器退出后的独立预览。

## 官方参数

实际内参和外参由设备标定数据提供，可通过 `get_intrinsics()`、`get_extrinsics_to()` 读取；支持的流格式可通过 `get_stream_profiles()` 或 `rs-enumerate-devices` 枚举。SDK 的高级模式 JSON 视觉预设用于成像和后处理，不是出厂标定文件。

# 机器人数据可视化与录制

`joint_monitor.py`（公开入口 `robot_data_web.py`）提供统一的机器人数据网页：29 关节、双 IMU、目标与曲线，以及深度实时预览与**数据录制**。只订阅 DDS，不发布 LowCmd、不释放运动服务，也不能发送运动指令。

## 相机源

| `--source` | 数据来源 | 场景 |
| --- | --- | --- |
| `sim` | 仿真进程内注入（`sim.py --profile door` 自动托管，无需单独启动） | door 档案仿真 |
| `realsense` | 实机 D435（pyrealsense2，读 `--camera-config` 相机模块） | 独立预览/录制，可与 DWaQ 同时启动 |
| `mock` | 合成斜坡深度 | 无设备开发验证 |
| `none` | 无相机 | 纯关节监控；仿真直接运行 Python 时的默认值 |

相机模块来自 `profiles/configs/cameras/*.json`。Door 仿真使用 `d435_rgbd.json`（640×480@30）；实机短命令默认用 480×270@60 的仅深度配置。实机单 D435 的深度流互斥规则见 [相机模块说明](../../profiles/configs/cameras/README.md)。

Monitor 只显示深度，即使相机配置中有 `rgb`，也不打开 RealSense 的色彩流。DWaQ 不打开相机，Parkour 和 Hybrid 实机控制器则会占用同一台 D435。

## 启动

配置实机网卡后，用 `./g1 monitor` 开启 D435 深度、关节和 IMU 监控。Parkour 或 Hybrid 占用相机时，用 `./g1 monitor --joints-only`。网卡和网页地址见 [实机部署](../../docs/ROBOT_SETUP.md)。

```bash
# 仿真：sim.py --profile door 自动托管数据网页，无需单独启动；
# 浏览器打开 http://127.0.0.1:8765/
./g1 monitor --interface eth0  # 使用实际 DDS 网卡，仅开启深度流
.venv/bin/python apps/monitor/robot_data_web.py --source mock                   # 无设备验证
bash apps/monitor/monitor.sh eth0 0.0.0.0 8765                         # 旧行为：纯关节监控（域 0）
```

相机出帧后，可用网页顶部的 `开始录制/停止录制` 按钮（POST `/api/record/start|stop`），也可在启动时加 `--record`。

## 录制格式

每次录制一个目录 `logs/recordings/<来源>-<时间戳>/`（仿真与实机同格式）。不保存 RGB 或任何 MP4，深度伪彩仅用于网页实时预览：

- `pointcloud/NNNNNN.ply`：**PLY 二进制点云**。每个成功保存的深度样本对应一个文件；按内参反投影为 xyz（深度光轴系 x 右/y 下/z 前），无效深度不生成点。RealSense 点云来自滤波后的深度，不是未处理的传感器数据。
- `timestamps.csv`：用 `index` 对应 PLY 文件名。用 `host_monotonic` 计算帧间隔；另保留源时间、应用序号和 RealSense 原始设备帧号。
- `model/NNNNNN.png`：仅当仿真帧带策略深度输入时保存，编号对应原始样本。
- `state.csv`：关节/IMU/目标快照（监控采样率）；
- `meta.json`：来源、内参、配置帧率 `configured_fps`、实际点云保存帧率 `saved_fps`、PLY 写入耗时及 RealSense 采集统计。

离线读取示例：

```python
import open3d as o3d
cloud = o3d.io.read_point_cloud("pointcloud/000000.ply")
```

点云不会补帧，也不保证达到相机标称帧率。例如，设备配置为 60 Hz，不代表每秒保存了 60 个点云。仿真还受渲染和投递节奏影响。旧录像的 `host_monotonic` 在写盘后记录；新录像在取帧后、滤波前记录（仿真为投递时），更适合与同一主机上的状态数据对齐。

### 实机帧率诊断

关掉其它相机程序，用部署环境的 Python 在 PC2 上运行仅取帧测试：

```bash
LD_LIBRARY_PATH="$PWD/.deps/realsense/lib:$PWD/.deps/runtime/python/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  .deps/python/bin/python3 apps/monitor/probe_realsense.py \
  --camera-config profiles/configs/cameras/d435_depth_480x270_60hz.json --seconds 30
```

随后用同一配置分别运行 Monitor 预览和至少 30 秒的点云录制。对比 `/api/status` 中的实收帧率、原始设备帧号缺口和滤波耗时，以及 `meta.json` 中的保存帧率与 PLY 写入耗时。应用序号连续不代表设备没有漏帧。

## 端点

- `/`：统一网页；`/api/joints`、`/api/stream`（SSE 关节流）、`/api/status`（相机+录制状态）；
- `/depth.mjpg`：MJPEG 深度实时流；Parkour 仿真使用 `/raw.mjpg` 和 `/model.mjpg`。
- `POST /api/record/start|stop`：录制开关。

## 依赖

需要 Python、NumPy、CycloneDDS、官方 `unitree_sdk2py`、OpenCV（headless 即可）与可选 `pyrealsense2`，版本范围见 `requirements.txt`。C++ 控制器本身不依赖这些 Python 包。解释器与运行库解析规则见 [网页监控部署](../../docs/WEB.md)。

硬件边界见 [硬件控制说明](../../docs/HARDWARE.md)。

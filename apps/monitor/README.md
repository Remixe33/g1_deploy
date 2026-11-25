# 机器人数据可视化与录制

`joint_monitor.py`（公开入口 `robot_data_web.py`）提供统一的机器人数据网页：29 关节、双 IMU、目标与曲线，外加相机 RGB/Depth 实时流与**数据录制**。只订阅 DDS，不发布 LowCmd、不释放运动服务，也不能发送运动指令。

## 相机源

| `--source` | 数据来源 | 场景 |
| --- | --- | --- |
| `sim` | 仿真进程内注入（`sim.py --profile door` 自动托管，无需单独启动） | door 档案仿真 |
| `realsense` | 实机 D435（pyrealsense2，读 `--camera-config` 相机模块） | 独立预览/录制，可与 DWaQ 同时启动 |
| `mock` | 合成斜坡深度 + 彩条 | 无设备开发验证 |
| `none` | 无相机 | 纯关节监控；仿真直接运行 Python 时的默认值 |

相机模块来自 `profiles/configs/cameras/*.json`。Door 仿真使用 `d435_rgbd.json`（640×480@30）；实机短命令默认用 480×270@60 的仅深度配置。实机单 D435 的深度流互斥规则见 [相机模块说明](../../profiles/configs/cameras/README.md)。

在 G1 机载开发机 PC2 上启动实机监控后，网页可独立显示通过 DDS 收到的关节和 IMU 状态，以及本机 D435 的画面。使用 `d435_rgbd.json` 时，网页收到彩色帧后显示 RGB 窗格。DWaQ 不打开相机，Parkour 和 Hybrid 实机控制器则会占用同一台 D435。

## 启动

配置实机网卡后，用 `./g1 monitor` 开启 D435 深度、关节和 IMU 监控。Parkour 或 Hybrid 占用相机时，用 `./g1 monitor --joints-only`。原厂图传可能占用色彩节点；只有确认相机空闲后才单独测试 RGB-D。网卡和网页地址见 [实机部署](../../docs/ROBOT_SETUP.md)。

```bash
# 仿真：sim.py --profile door 自动托管数据网页，无需单独启动；
# 浏览器打开 http://127.0.0.1:8765/
./g1 monitor --interface eth0 --camera-config profiles/configs/cameras/d435_rgbd.json  # 实机 RGB-D（显式启用）
.venv/bin/python apps/monitor/robot_data_web.py --source mock                   # 无设备验证
bash apps/monitor/monitor.sh eth0 0.0.0.0 8765                         # 旧行为：纯关节监控（域 0）
```

录制触发：网页按钮 `开始录制/停止录制`（POST `/api/record/start|stop`），或启动时加 `--record`。

## 录制格式

每次录制一个目录 `logs/recordings/<来源>-<时间戳>/`（仿真与实机同格式同代码）：

- `rgb.mp4`：RGB 视频（mp4v）；
- `depth_color.mp4`：深度伪彩视频（0–2.5 m TURBO 伪彩）；
- `pointcloud/NNNNNN.ply`：**PLY 二进制点云**（每点 xyz+RGB，深度光轴系 x 右/y 下/z 前）——按相机模块内参逐帧反投影，全帧率保存（640×480 约 110 MB/s，录制时长自控）；CloudCompare/Open3D/PCL 直接打开；
- `timestamps.csv`：每帧主机/源时间戳与序列号（丢帧标记 gap）；
- `state.csv`：关节/IMU/目标快照（监控采样率）；
- `meta.json`：来源、相机模块、内参、帧计数、序列缺口等。

离线读取示例：

```python
import numpy as np
dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                  ("red", "u1"), ("green", "u1"), ("blue", "u1")])
cloud = np.fromfile("pointcloud/000000.ply", dtype=dtype)
```

按 sequence 去重，同一帧的重复投递不入库。仿真录制帧率受查看器循环节流（约 14 fps）；实机为相机帧率。

## 端点

- `/`：统一网页；`/api/joints`、`/api/stream`（SSE 关节流）、`/api/status`（相机+录制状态）；
- `/rgb.mjpg`、`/depth.mjpg`：MJPEG 实时流；
- `POST /api/record/start|stop`：录制开关。

## 依赖

需要 Python、NumPy、CycloneDDS、官方 `unitree_sdk2py`、OpenCV（headless 即可）与可选 `pyrealsense2`，版本范围见 `requirements.txt`。C++ 控制器本身不依赖这些 Python 包。解释器与运行库解析规则见 [网页监控部署](../../docs/WEB.md)。

接口为 `/`、`/api/joints`、`/api/stream`、`/api/status`、`/rgb.mjpg`、`/depth.mjpg`、`/api/record/*`。硬件边界见 [硬件控制说明](../../docs/HARDWARE.md)。

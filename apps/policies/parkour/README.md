# Parkour 深度感知行走控制器

`g1_parkour` 使用深度编码器和动作网络进行双 ONNX 推理，支持 MuJoCo 同步深度观测和实机 RealSense 深度输入。

## 常用命令

```bash
./g1 parkour check               # 双网络离线检查，无需相机或 DDS
./g1 parkour sim --domain 1     # 仿真，需保留深度观测
./g1 parkour real eth0          # 实机，自动管理相机，默认空跑
```

也可使用 `bash apps/policies/parkour/parkour.sh check|sim|real ...`。直接运行构建产物：

```bash
./build/g1_parkour --sim --domain 1 --interface lo
./build/g1_parkour --hardware --domain 0 --interface eth0
```

## 策略结构

```text
8 帧本体状态：8 × 96 = 768
8 张 18×32 深度图 → 深度编码器 → 128 维地形特征
768 + 128 = 896 → 动作网络 → 29 个关节动作
```

- 本体历史按观测项排列，深度历史按完整图像排列，均从旧到新。
- Parkour 使用 `rt/secondary_imu` 的 torso IMU，不回退到 pelvis IMU。
- 仿真通过 `/dev/shm/g1-observation-<UID>` 共享内存读取同步状态和深度。
- 实机通过 C++ RealSense 采集线程读取 Z16，转换为米制深度后在 50 Hz 策略线程预处理。

## 配置

- `config/g1_parkour.json`：模型路径、关节映射、默认姿态、PD 参数、动作转换、深度历史和硬件限制。
- `config/realsense_parkour.json`：实机深度预处理和相机外参。
- `models/0-depth_encoder.onnx`：深度编码器。
- `models/actor.onnx`：动作网络。
- [`d435_depth_480x270_60hz.json`](../../../profiles/configs/cameras/d435_depth_480x270_60hz.json)：Parkour 相机流配置。

`reset_seconds` 默认 3 秒，策略周期为 20 ms。修改配置后需重启。

## 输入与控制

键盘：`9` 接管、`p` 复位、`r` 策略、`w/s` 前进速度、`q/e` 转向、`x` 清零、空格停止。

仿真启动时自动识别唯一兼容手柄，也可添加 `--joystick PATH`。手柄映射为 `Start/X/A/Y/B`，不支持热插拔重连。

实机使用原配遥控器，按键同样为 `Start/X/A/Y/B`，也可按 `Select` 停止退出。空跑为 `X → 等待 → A`，真实控制为 `Start → X → 等待复位 → A`。实机前须核对相机安装、torso IMU 轴向、关节方向和硬件参数。

## 相机与运行注意

- 实机控制器不依赖 Python 相机进程；启动前退出独立 RealSense 网页，避免 USB 占用。
- 实机策略深度为 480×270 Z16、60 Hz；预处理参数来自 `realsense_parkour.json`。
- 控制线程按 2 ms 调度，策略线程按 20 ms 调度；首帧、输入或结果达到 60 ms 未更新会停止。
- `./build/g1_camera_check` 只读验证相机，不初始化 DDS 或发布电机命令。
- 没有 RealSense C++ SDK 时仍可构建仿真和离线检查；实机启动会明确报错。

相机链路、深度历史和模型来源详见 [Parkour 与 RealSense 说明](../../../docs/PARKOUR.md)，硬件接管见 [硬件控制说明](../../../docs/HARDWARE.md)。

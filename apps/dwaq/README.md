# DWaQ 行走控制器

`g1_dwaq` 使用单个 ONNX 网络控制 G1 的 29 个关节，支持离线检查、MuJoCo 仿真和实机低层控制。核心控制链路不依赖 Python、ROS 或 MuJoCo。

## 常用命令

在源码根目录或安装目录执行：

```bash
./g1 dwaq check                 # 离线加载模型并推理一次，不连接 DDS
./g1 dwaq sim --domain 1       # 仿真，需先启动 MuJoCo
./g1 dwaq real eth0            # 实机，替换为实际网卡；默认空跑
```

辅助检查：

```bash
./g1 interfaces                # 网卡与到控制板的路由
./g1 joystick                  # 自动识别兼容手柄，仅检查输入
./g1 read-only eth0            # 只读 LowState、CRC、机型、电机和主 IMU
```

直接运行构建产物时使用长参数：

```bash
./build/g1_dwaq --check-policy --config apps/dwaq/config/g1_locomotion.json
./build/g1_dwaq --sim --domain 1 --interface lo \
  --config apps/dwaq/config/g1_locomotion.json
./build/g1_dwaq --hardware-read-only --domain 0 --interface eth0 \
  --config apps/dwaq/config/g1_locomotion.json
```

## 输入与控制

### 键盘

- `9`：仿真接管；实机还需通过硬件检查并释放官方运动服务。
- `p`：从当前实测角度复位。
- `r`：复位完成后进入策略。
- `w/s`：前后速度；`a/d`：横向速度；`q/e`：转向。
- `x`：速度清零；空格或 Ctrl+C：停止退出。

仿真输出操作顺序为 `9 → p → 等待 Holding default pose → r`。实机空跑时不按 `9/Start`，直接 `p → 等待 → r`。

### 手柄

仿真启动时自动识别唯一兼容手柄，无需 `--joystick`；检测到多个兼容手柄时要求显式选择，无兼容手柄时使用键盘。

```bash
./g1 dwaq sim --domain 1 --joystick /dev/input/js0  # 可选：显式指定
```

按键为 `Start` 接管、`X` 复位、`A` 策略、`Y` 清零、`B` 停止。进入策略前摇杆回中，之后直接使用摇杆，无需按住 RB。

自动识别只在启动时执行，不支持热插拔重连；拔出或读取失败仍进入停止流程。实机只使用宇树原配遥控器：`Start/X/A/Select`。

## 策略接口

当前模型来自训练运行 `2026-11-13_10-08-23`，接口为：

```text
observation [1,96] + observation_history [1,480] → actions [1,29]
```

每帧 96 维由角速度、重力方向、三维速度指令、关节位置、关节速度和上次动作组成；保留五帧完整观测，按 `history_order: time` 从旧到新排列，兼容旧 `term` 布局。

策略使用 LowState 主 IMU（pelvis）的机体系角速度和重力方向，不读取深度。输入与输出在策略关节顺序和 DDS 电机顺序之间转换。

## 配置

主配置为 `config/g1_locomotion.json`：

- `policy_path`：ONNX 模型路径，相对配置文件目录解析。
- `step_dt`：策略周期，默认 20 ms。
- `joint_ids_map`：策略关节到 DDS 电机的映射。
- `default_joint_pos`、`stiffness`、`damping`：默认姿态及 PD 参数。
- `observations`：观测缩放、限幅和历史布局。
- `actions`：动作缩放、偏置和反馈方式。
- `commands`：速度范围。
- `hardware`：关节限界、状态超时、温度、倾角和停止阻尼。
- `reset_seconds`：复位插值时长，默认 3 秒。

模型与配置来源及 SHA256 见 [../../ORIGIN.md](../../ORIGIN.md)。

## 运行与安全边界

- 启动默认空跑，不向真实电机发布命令。
- 接管会清除旧预览和策略历史，必须重新复位。
- 策略线程默认 50 Hz，控制线程按 2 ms 调度；推理期间不持目标缓存锁。
- 首帧、推理结果或输入快照达到三个策略周期未更新时停止。
- 停止时先执行阻尼，再等待策略线程退出。
- `mode_machine=5`、CRC 和硬件型号检查不得通过修改常量绕过。

详见 [硬件控制说明](../../docs/HARDWARE.md)。源码结构为 `src/main.cpp`（入口）、`src/options.cpp`（参数）、`src/controller.cpp`（控制循环）和 `include/g1_locomotion_policy.h`（策略）。

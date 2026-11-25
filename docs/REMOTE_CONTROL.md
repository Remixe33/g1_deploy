# G1 远端观测与目标跟随

PC2 通过回环 TCP 服务发送米制深度与近似配对的 DDS 状态；工控机通过 SSH 转发返回带会话和观测序号的目标。桥不发电机指令。DWaQ 控制程序通过当前用户的私有 Unix 数据报套接字取目标，控制循环不会等待网络。Monitor 仍只负责查看和录制。

## Door 仿真

从源码目录运行，终端一启动场景并创建私有目录：

```bash
remote_dir="$(mktemp -d)"
printf '%s\n' "$remote_dir"
./g1 task door sim scene --remote-socket "$remote_dir/target.sock"
```

终端二使用上一步打印的实际目录：

```bash
./g1 task door sim policy --remote-socket /tmp/实际目录/target.sock
```

在控制器终端按 `9` 启用仿真输出，再按 `p`、等复位结束、按 `r`。走动前在 MuJoCo 窗口按 `9` 解除仿真吊带。然后在第三个终端启动远端客户端，回控制器终端按 `g` 授权；客户端连接后先静止 3 秒，再低速前进 2 秒，然后归零。用 `--preset turn` 测转向，或用默认的 `stationary` 先检查链路。

```bash
.venv/bin/python apps/remote_control/client.py --preset walk
```

按 `x` 只锁定远端目标，空格或 Ctrl+C 退出控制器。网络断开、目标过期或拒绝后，先重新连接客户端，再到控制器终端按 `g`。网络消息不能复位、使能或解锁。可在 Monitor 查看仿真与空跑目标；桥不要求启用 Monitor（可用 `--no-data-web`）。

## 目标模式

- 默认 `velocity`：用机载 DWaQ，接收机身速度 `vx/vy/wz`（前、左、逆时针；m/s、rad/s）。机载限幅为 `|vx| ≤ 0.10`、`vy = 0`、`|wz| ≤ 0.15`。断线时速度归零，但零速度目标并不等于已验证机器人停稳。
- 场景桥和控制器都加 `--remote-control joints`（实机独立桥对应 `--command joints`），绕过 DWaQ ONNX，直接接收全身 29 关节 `q` 与 `dq`（DDS 顺序，rad、rad/s）。使用本地策略配置的复位姿态、位置限位和 PD 增益，`|dq| ≤ 1 rad/s`。不接受网络下发的力矩、增益或控制模式。断线时撤销全身参考，使用当前测量位置作为零速度保持参考；这只是仿真和空跑的行为，不是实机运动安全认证。
- 控制器加 `--fixed-joints 文件.json` 可锁定选定的 DDS 关节位置；文件是索引到弧度的映射，如 `{"12": 0.0}`。有有效远端目标时覆盖该关节的位置和速度；远端目标失效后回到当前模式的本地回退。先在仿真验证关节锁定与步态是否兼容。
- `--preset hold` 仅适用于 `joints`，返回首帧测量姿态，仅用于仿真吊带或空跑检查接口；它没有平衡反馈，不能代替站立控制器。自定义远端策略用 `--planner 模块:函数`，函数接收 `(metadata, depth)` 并返回 `{"vx": ..., "vy": ..., "wz": ...}` 或 `{"q": [...], "dq": [...]}`，由客户端附上会话、序号与耗时。这样可替换策略而不改观测传输或电机接口。

## 实机只能空跑

同一账户在 PC2 创建私有目录；分别在两个终端使用其绝对套接字路径。将 `eth0` 换成实际 DDS 网卡。

```bash
./install/g1 task door real --remote-socket /tmp/实际目录/target.sock

LD_LIBRARY_PATH="$PWD/.deps/realsense/lib:$PWD/.deps/cyclonedds/lib:$PWD/.deps/runtime/python/lib" \
  .deps/python/bin/python3 -u install/apps/remote_control/bridge.py \
  --interface eth0 --target-socket /tmp/实际目录/target.sock
```

远端模式禁止实机接管，按 `Start` 或 `9` 都不会创建 `rt/lowcmd` 发布者或释放原厂运动服务。用遥控器 `X` 复位空跑目标，等待完成后按 `A`，再在控制器终端按 `g`。遥控器 `Y` 撤销远端目标，`B` 或 `Select` 停止。PC2 上不要同时启动另一相机采集进程；需要看状态时可运行 `./install/g1 monitor --joints-only`。

在工控机建立隧道，并用同一份源码下的客户端（Python 和 NumPy）运行：

```bash
ssh -N -L 127.0.0.1:8770:127.0.0.1:8770 unitree@192.168.123.164
.venv/bin/python apps/remote_control/client.py --preset stationary
```

两条命令分别在不同终端运行。连续空跑 30 秒，检查有效观测频率、往返延迟、目标年龄和拒绝原因；用断开隧道、拖延响应和重连验证不会自动恢复。日志的往返延迟与目标年龄由 PC2 单调时钟计算，不依赖两机对时。禁止把本模式作为实机运动或开门验收。

## 数据接口

版本 1：每个 TCP 消息以 4 字节网络字节序长度开头。观测体包含 4 字节 JSON 长度、UTF-8 JSON 和行优先的小端 `float32` 米制深度。元数据包括 `version`、`session`、`sequence`、`capture_ns`、`state_ns`、`shape`、`dtype`、`intrinsics`、`state`、`command`。状态包含 DDS 顺序的 `q/dq`、骨盆 `imu_wxyz`、`gyro_rad_s` 和未去重力的 `accelerometer_m_s2`；深度与最近的主机接收状态最多相差 50 ms，不是硬件同步。返回体仅 UTF-8 JSON。观测最多 10 Hz、一次仅一个待回复观测，较旧深度会被覆盖；目标年龄上限为 250 ms。

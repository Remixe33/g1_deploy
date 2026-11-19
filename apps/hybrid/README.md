# Hybrid 混合行走控制器

`g1_hybrid` 在同一控制器中组合 DWaQ 和 Parkour：站立、后退和原地转向使用 DWaQ，持续前进时切换到 Parkour。

## 常用命令

```bash
./g1 hybrid check               # 验证三个网络及双向切换
./g1 hybrid sim --domain 1     # 仿真，需保留深度观测
./g1 hybrid real eth0          # 实机，自动管理相机，默认空跑
```

手柄自动识别与 DWaQ/Parkour 相同，也可用 `--joystick PATH` 显式指定。实机只使用原配遥控器。

## 切换规则

默认配置位于 `config/g1_hybrid.json`：

- 前进速度持续达到 `0.15 m/s` 满 `0.2 s`：进入 Parkour。
- 速度降到 `0.05 m/s` 或后退：回到 DWaQ。
- 两阈值之间保持当前模式，避免反复切换。
- 切换不做角度或 PD 增益混合，也不清空策略历史。

每个 20 ms 周期都依次运行两套策略并更新双方历史，只发布当前选中策略的输出。后退时 DWaQ 接收负前进速度，Parkour 使用零前进速度。

## 初始化与运行

- 启动时在 DDS 和输出端初始化前预热三个 ONNX 网络，默认 `warmup_steps: 10`。
- 预热使用虚拟站姿和深度图，结果全部丢弃，随后清空历史并正式从 DWaQ 开始。
- 复位目标采用 DWaQ 默认姿态。
- 共用 Parkour 的 DDS、输入、监控、复位和停止循环，同一时刻只有一个输出端。
- 即使站立或后退，相机和观测链路也必须在线；数据过期会停止，不自动降级。

`policies` 指向 DWaQ 和 Parkour 的配置，`switching` 设置切换阈值，`reset_seconds` 与 `hardware` 由 Hybrid 自己维护。两份模型仍使用各自的关节映射、归一化和 PD 增益。

操作键与 Parkour 相同。验证结果仅覆盖已执行场景，不能外推为障碍物或实机切换验收。

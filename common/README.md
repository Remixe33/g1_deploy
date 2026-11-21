# 公共控制底座

`common/include/` 提供跨控制器共用的 header-only 模块。控制器专属代码放在 `apps/policies/<程序>/`，公共实现不复制到各任务中。

## 控制结构

```text
DDS / 相机 / 输入
  → 带时间戳的状态快照
  → PolicyWorker 策略线程（默认 50 Hz）
  → 最新关节目标
  → 控制线程（2 ms / 500 Hz）
  → rt/lowcmd
```

策略邮箱只保留最新输入和结果，推理期间不持锁。首帧、输入快照或结果达到三个策略周期未更新时触发停止；错过周期时跳过，不集中补发。实际频率受系统调度和 DDS 耗时影响，不是硬实时保证。

## 主要模块

- `g1_control_options.h`：运行参数和模式校验。
- `g1_control_profile.h`：任务档案解析、程序守卫和相机兼容检查。
- `g1_config_utils.h`：配置数组和相对路径。
- `g1_onnx_utils.h`：ONNX 接口校验与执行。
- `g1_observation_history.h`：观测历史更新和布局。
- `g1_policy_worker.h`：策略线程、最新结果和看门狗。
- `g1_loop_scheduler.h`：固定周期调度。
- `g1_dds_state_cache.h`：DDS 订阅、快照和故障锁存。
- `g1_joint_mapping.h`：关节名称和 DDS 顺序转换。
- `g1_control_targets.h`：复位插值、位置目标和阻尼目标。
- `g1_motor_channel.h`：LowCmd 发布及竞争发布者检测。
- `g1_monitor_publisher.h`：使能、实际目标和空跑遥测。
- `g1_gamepad.h`：手柄映射、输入诊断和启动时自动识别。
- `g1_unitree_remote.h`：宇树原配遥控器。
- `g1_hardware.h`：CRC、硬件状态、限位和指令检查。
- `g1_async_log.h`：有界后台日志，避免慢终端阻塞控制。

## 输入与安全状态

控制器统一按“停止 → 遥控事件 → 键盘事件”的优先级处理输入。启动默认空跑，实机只有显式 `9/Start` 后才创建 LowCmd 发布端并申请接管。

接管会清空旧预览和策略状态，必须重新从实测角度复位。复位完成且姿态满足要求后才能进入策略。停止时先执行阻尼，再结束策略线程；退出不自动恢复官方运动模式。

手柄自动识别仅在启动时执行，不支持热插拔重连；拔出或读取失败仍停止。实机只使用原配遥控器。

具体硬件约束见 [硬件控制说明](../docs/HARDWARE.md)，各策略的观测和动作配置仍归对应应用所有。

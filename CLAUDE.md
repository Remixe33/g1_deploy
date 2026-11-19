# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概览

Unitree G1 29DOF 人形机器人的机载部署仓库（C++17）。三个行走控制器：`g1_dwaq`（DreamWaQ 单网络）、`g1_parkour`（深度编码器+动作网络双 ONNX）、`g1_hybrid`（按前进速度在两者间切换）。另有只读 Python 网页工具（关节监控、RealSense 预览）和 MuJoCo 仿真启动器。C++ 控制链路不依赖 ROS、MuJoCo 或 Python。仓库文档与代码注释以中文为主，新增内容沿用中文。

## 构建（无自动化测试）

```bash
export ONNXRUNTIME_ROOT=/path/to/native/onnxruntime   # 必须与目标 CPU 架构匹配（aarch64/x86_64）
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
```

依赖：Unitree SDK2（`/usr/local` 或 `/opt/unitree_robotics`）、Boost、yaml-cpp、OpenCV（core/imgproc/photo）、CPU 版 ONNX Runtime。`realsense2` C++ SDK 自动检测：缺省时仿真与离线检查仍可构建，实机相机报错；机器人构建加 `-DG1_REQUIRE_REALSENSE=ON`。仅装 pyrealsense2 不满足 C++ 链接。

**本仓库刻意不构建自动化测试**（2026-10-22 移除了全部测试与 CTest 目标，见 CHANGELOG）。验证手段是离线检查入口，不要主动重建测试框架：

```bash
./g1 dwaq check          # 离线加载模型推理一次，不连 DDS（parkour/hybrid 同理）
./build/g1_camera_check  # 只读验证 RealSense 采集，不碰 DDS
./g1 read-only <网卡>    # 只读 LowState/CRC/机型/电机/IMU 检查后退出
./g1 interfaces          # 列出网卡与到控制板的路由
```

## 常用运行命令

统一入口 `./g1 <dwaq|parkour|hybrid> <check|sim|real>`，源码根目录和 `install/` 均可用：

- `./g1 dwaq sim --domain 1` / `./g1 parkour sim --domain 1` / `./g1 hybrid sim --domain 1` —— 仿真（先启动 MuJoCo，见下）；`real <网卡>` —— 实机（域 0，默认空跑）。仿真启动时自动识别唯一兼容手柄，`--joystick PATH` 可覆盖；不支持热插拔重连，实机仍只用原配遥控器。
- 仿真：`python3 apps/simulation/sim.py --mujoco-root /path/to/unitree_mujoco_sim --domain 1`（外部仓库，不在本仓库内；`--terrain`、`--follow`、`--depth-preview` 见 README）。仿真与控制器须同机同用户，一次只运行一个控制器。
- 网页监控：`bash apps/monitor/monitor.sh <网卡> [HOST] [PORT]`；相机预览：`bash apps/realsense/realsense.sh --host 0.0.0.0 --port 8766`（实机控制前必须退出，避免占用 USB）。
- 机器人一键配置：`bash scripts/setup_robot.sh`（ARM64 机载，离线装 Python/SDK/ORT 后编译安装）。
- 直接跑构建产物用长参数，如 `./build/g1_dwaq --check-policy --config apps/dwaq/config/g1_locomotion.json`、`--hardware --domain 0 --interface eth0`。

## 架构

### 目录与入口分工

- `apps/<程序>/{src,include,config,models}` —— 程序专属代码。每个 C++ 程序固定三文件：`main.cpp`（入口/帮助/异常出口）、`options.cpp`（命令行解析与校验）、`controller.cpp`（离线检查、初始化、控制循环）。
- `common/include/` —— 跨程序共用的 **header-only** 模块（无 .cpp，直接被各 target include）：`g1_loop_scheduler.h`（固定周期网格、跳过错过的 tick）、`g1_policy_worker.h`（策略线程邮箱：只保留最新输入/结果，推理期间不持锁）、`g1_dds_state_cache.h`（DDS 订阅、带时间戳快照、故障锁存）、`g1_control_targets.h`（复位插值、目标填充）、`g1_joint_mapping.h`（关节映射）、`g1_onnx_utils.h`、`g1_observation_history.h`、`g1_config_utils.h`（读 JSON、相对路径解析）、输入/监控/日志等。跨程序共用代码放 `common/`，禁止复制多份。
- `apps/realsense/src/g1_depth_camera.cpp` —— C++ 原生 RealSense 采集（静态库 `g1_depth_camera`，被 parkour/hybrid 链接）；`apps/simulation/` —— 任务档案驱动的 MuJoCo 启动、统一 Raw/可选 Model/可选 RGB 相机管线及 Parkour 共享内存桥接；door 的独立 RGB/Depth 预览不依赖 Parkour 策略配置，RGB 外参由相机模块提供。`apps/monitor/`、`apps/realsense/` 的网页服务是独立 Python 进程，只读。

### 双线程控制循环

控制线程 500 Hz（2 ms，`g1_loop_scheduler`）持续下发同步缓存中的最新关节目标；策略线程按各策略周期（默认 20 ms/50 Hz）做预处理+ONNX 推理，经 `g1_inference::PolicyWorker` 邮箱交换快照，推理时不持缓存锁。首结果到达前保持站姿；首帧等待/结果年龄/输入快照年龄超 60 ms 或线程异常走停止流程；错过周期跳过，不突发补发。Hybrid 每周期依次前推两套策略、持续更新双方观测/动作历史，只发布当前选中策略的输出；切换阈值 0.15/0.05 m/s 带滞回，切换不混合、不清空历史。

### DDS 与关节顺序

- 实机域 0（连控制板的网卡，绝不用 `lo`），仿真用正数域 + `lo`，两端域号必须一致。
- 话题：`rt/lowstate`（关节+主 IMU，PR 坐标）、`rt/lowcmd`（仅接管后创建）、`rt/g1/dry_run_target`（空跑预览遥测）、`rt/secondary_imu`（Parkour 的 torso IMU，缺失则报错退出，不回退 pelvis）。
- 两种关节顺序：DDS 顺序（`g1_joints::kDdsNames` 29 关节）与各策略自己的顺序（JSON `joint mapping`）。输入和增益在填充边界统一转换到 DDS 顺序。

### 空跑/接管状态机（所有控制器一致）

启动即空跑：不按 `9`/Start 不创建 `rt/lowcmd` 发布器、不释放官方运动服务。接管 = 完整硬件检查 → MotionSwitcher 释放原服务 → 确认无竞争发布者 → 阻尼等待；接管清空空跑目标与策略历史，必须重新 `p`/X 复位（从实测角插值）后才能 `r`/A 进策略。停止（空格/Ctrl+C/SIGTERM/Select）在已接管时先发约 0.5 s 阻尼再退出；退出不恢复官方运动模式。

### 配置体系

每个程序自己的 JSON：`apps/dwaq/config/g1_locomotion.json`、`apps/parkour/config/g1_parkour.json` + `realsense_parkour.json`（深度预处理/相机外参）、`apps/hybrid/config/g1_hybrid.json`（`policies` 指向两份策略 JSON，相对混合配置目录解析）。模型路径写在 JSON 内（`policy_path` / `models.*`），相对路径以 JSON 所在目录为基准。`reset_seconds`、`hardware`（关节限界、`limit_target_position` 等，DDS 顺序）各程序独立维护；配置仅在启动时读取，改后须重启。

### 深度链路（Parkour/Hybrid）

实机：RealSense Z16 480×270@60Hz → C++ 采集线程转米制 float32 → 进程内最新帧缓存 → 50 Hz 推理线程做缩放/裁剪/补洞/归一化（参数 `realsense_parkour.json`）。不使用 Python、UDP、HTTP 或跨进程共享内存传深度。仿真：外部 `unitree_mujoco` Python 端写 `/dev/shm/g1-observation-<UID>` 共享内存，控制器自动连接。

## 安全边界（改硬件相关代码前必读 docs/HARDWARE.md）

- 这是真机部署仓库：**不要对实机运行 `real`/`run` 模式，除非用户明确提出**；验证一律用 `check`/`read-only`/`camera_check`/仿真。
- `mode_machine=5`、CRC、机型等检查常量是刻意的硬件约束，不得修改常量绕过。
- 刻意不做的保护（不要"顺手加回"）：估算 PD 力矩检查、实测关节越界自动退出、目标变化率限制——均为 2026-10-19 按操作者要求移除，见 CHANGELOG。
- 仿真不因躯干高度/倾角停止（允许跌倒调试）；真机保留倾角检查。

## 文档地图

README.md 是项目导航和通用构建/启动入口；各目录的 README.md 分别维护控制器、仿真、相机、任务档案和公共底座细节。docs/HARDWARE.md（接管/停止语义与硬件检查边界）、docs/PARKOUR.md（相机链路对齐 Instinct、深度历史、验证范围）、docs/ROBOT_SETUP.md（机载一键配置）、docs/WEB.md（网页依赖）保留专项说明；CHANGELOG.md 记录历史验证与行为变更（改保护逻辑前先查）；ORIGIN.md 记录模型/配置来源与 SHA256——更换模型或配置后应更新其中的哈希记录。

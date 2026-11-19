# g1_deploy — G1 机载部署

## 简要介绍

面向宇树 G1 29 自由度人形机器人的 C++17 部署项目。直接通过 Unitree DDS 读取低层状态并下发关节目标，使用 ONNX Runtime 执行预训练策略。

核心控制链路不依赖 ROS、Python 或 MuJoCo；Python 仅用于仿真、只读网页监控和独立相机预览。当前包含三个行走控制器（DWaQ、Parkour、Hybrid）、任务档案组合机制、MuJoCo 仿真启动器（含 door 任务场景与 RGB/Depth 双视角预览）和只读网页监控。

### 模块与文档导航

| 模块 | 功能 | 专题文档 |
| --- | --- | --- |
| DWaQ | 单网络行走、复位、离线检查和手柄诊断 | [apps/dwaq/README.md](apps/dwaq/README.md) |
| Parkour | 深度编码器＋动作网络双 ONNX 推理 | [apps/parkour/README.md](apps/parkour/README.md) |
| Hybrid | DWaQ 与 Parkour 按前进速度切换 | [apps/hybrid/README.md](apps/hybrid/README.md) |
| 仿真 | 外部 MuJoCo、地形、任务相机和 door | [apps/simulation/README.md](apps/simulation/README.md) |
| RealSense | C++ 深度采集、相机检查和独立网页预览 | [apps/realsense/README.md](apps/realsense/README.md) |
| 关节监控 | 只读关节、IMU、目标与空跑网页 | [apps/monitor/README.md](apps/monitor/README.md) |
| 任务档案 | 组合程序/策略、场景、机器人和相机 | [profiles/README.md](profiles/README.md) |
| 相机模块 | 流格式、视场、RGB–Depth 外参和官方参数 | [cameras/README.md](cameras/README.md) |
| 机器人模块 | 标准头部和后仰安装配置 | [robots/README.md](robots/README.md) |
| 公共控制底座 | 控制循环、DDS、推理线程、输入与硬件检查 | [common/README.md](common/README.md) |

补充文档：

- [硬件控制与验证边界](docs/HARDWARE.md)
- [Parkour 与 RealSense 详细说明](docs/PARKOUR.md)
- [网页监控部署](docs/WEB.md)
- [ARM64 机器人一键配置](docs/ROBOT_SETUP.md)
- [模型与资产来源](ORIGIN.md)
- [历史修改与验证记录](CHANGELOG.md)

## 文件架构

```text
apps/        各控制器、仿真、RealSense 和只读网页工具（每个目录附 README）
common/      公共控制与推理模块（header-only）
profiles/    任务档案：组合策略程序、场景、机器人与相机
cameras/     相机模块：流格式、视场和外参
robots/      机器人模块：标准头部与后仰安装
docs/        硬件、网页、部署和 Parkour 专项说明
scripts/     ARM64 机器人环境配置
cmake/       构建信息模板
build/       本机构建产物
install/     可部署安装目录
```

每个 C++ 控制器的源码入口固定为 `src/main.cpp`（入口）、`src/options.cpp`（参数解析）和 `src/controller.cpp`（控制循环）。跨程序实现放在 `common/`，不在任务间复制。任务与模块解耦：模块文件按自身命名，任务组合只发生在 `profiles/`。

## 安装说明

依赖：Linux、CMake≥3.16、C++17、Unitree SDK2、Boost、yaml-cpp、OpenCV 和 CPU 版 ONNX Runtime。RealSense C++ SDK 自动检测；实机相机构建建议加 `-DG1_REQUIRE_REALSENSE=ON`。

SDK、ONNX Runtime 和库文件必须与目标 CPU 架构匹配：

```bash
uname -m
export ONNXRUNTIME_ROOT=/path/to/native/onnxruntime
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
```

源码根目录和安装目录使用相同的 `./g1 ...` 命令。ORT 随包安装，SDK2/CycloneDDS、yaml-cpp 和系统运行库由目标环境提供。仿真启动器 `apps/simulation/sim.py` 不随安装包安装，需从源码目录运行。

```bash
ldd install/bin/g1_dwaq       # 不应出现 not found
cd install
./g1 check                    # 旧 DWaQ 离线入口，兼容保留
```

需要当前架构的压缩包时：

```bash
cd build
cpack -G TGZ
```

## 命令说明

统一入口 `./g1 <dwaq|parkour|hybrid> <check|sim|real>`，源码根目录和安装目录均可使用：

```bash
./g1 dwaq check        # 离线加载模型推理一次，不连 DDS（parkour/hybrid 同理）
./g1 dwaq sim --domain 1
./g1 dwaq real eth0    # 实机：DDS 域 0 + 连接控制板的网卡
```

`./g1 run eth0` 等旧命令仍兼容。网页监控与相机预览作为独立服务启动：

```bash
./g1 monitor eth0 [HOST] [PORT]     # 只读关节、IMU、目标与空跑网页
./g1 realsense [选项]               # 独立 RealSense 网页预览
```

任务档案把策略程序、场景、机器人与相机组合为一项任务：

```bash
./g1 dwaq sim --domain 1 --profile door
./g1 parkour sim --domain 1 --profile parkour
./g1 hybrid sim --domain 1 --profile hybrid
```

不带 `--profile` 时使用各程序原有默认配置。字段和兼容规则见 [任务档案说明](profiles/README.md)。

仿真启动器 `apps/simulation/sim.py` 的选项（`--profile`、`--terrain`、`--depth-preview`、`--fov-debug`、`--follow`、`--window-backend` 等）见 [仿真说明](apps/simulation/README.md)；仿真与控制器须同机同用户，一次只运行一个控制器。

只读检查与诊断：

```bash
./g1 interfaces        # 列出网卡与到控制板的路由
./g1 read-only eth0    # 只读 LowState/CRC/机型/电机/IMU 检查
./g1 joystick          # 手柄输入诊断
```

仿真启动时自动识别唯一兼容手柄，无需设置 `--joystick`；也可用该参数显式指定设备。没有手柄时使用键盘，多个兼容手柄时要求明确选择。自动识别只在启动时执行，不支持热插拔重连；实机只使用原配遥控器。

## 操作示例

### door 任务（仿真，两个终端）

终端一，任务档案驱动仿真（doors 场景 + 头部后仰 20° + RGB/Depth 双视角预览）：

```bash
.venv/bin/python apps/simulation/sim.py \
  --profile door --follow
```

终端二，DWaQ 行走控制：

```bash
./g1 dwaq sim --domain 1 --profile door
```

运行后 door 任务仿真默认使用 DWaQ 策略进行移动控制，仿真窗口显示独立 RGB/Depth 双视角。行走操作：`9/Start` 接管 → `p/X` 复位（约 3 s）→ `r/A` 进策略；`w/s/a/d/q/e` 调速转向，`x/Y` 速度清零，空格停止。

### Parkour / Hybrid（仿真）

```bash
.venv/bin/python apps/simulation/sim.py \
  --terrain mixed
./g1 parkour sim --domain 1
```

### 实机只读检查与空跑

```bash
./g1 interfaces
./g1 read-only eth0
```

只读模式不加载策略、不创建 LowCmd 发布器、不释放官方运动服务。控制器默认空跑；实机真实输出需要显式 `Start/9` 接管，并在接管后重新复位，停止时先发送阻尼命令。

## 注意事项

- 首次实机测试必须使用吊架和独立停止手段，实机相关边界见 [硬件控制说明](docs/HARDWARE.md)。
- 仓库不构建自动化测试；离线检查、空跑和仿真不能替代实机运动验收。
- 实机使用 DDS 域 `0` 和连接控制板的网卡，仿真使用正数域和 `lo`，两端域号必须一致。
- 各程序 JSON 配置仅在启动时读取，修改后须重启生效。
- D435 单深度传感器同一时刻只能开一路流，原生深度流与 Parkour 策略管线在实机互斥（详见 [相机模块说明](cameras/README.md)）。
- 仿真器已内置于 `third_party/unitree_mujoco`（来源见 [ORIGIN.md](ORIGIN.md)）；`--mujoco-root` 可指向其他等价检出路径。

# g1_deploy — G1 机载部署

面向宇树 G1 29 自由度人形机器人的 C++17 部署项目。直接通过 Unitree DDS 读取低层状态并下发关节目标，使用 ONNX Runtime 执行预训练策略。

核心控制链路不依赖 ROS、Python 或 MuJoCo；Python 仅用于仿真、只读网页监控和独立相机预览。

## 程序与文档导航

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

## 构建与安装

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

## 控制器命令

```bash
./g1 dwaq check
./g1 parkour check
./g1 hybrid check

./g1 dwaq sim --domain 1
./g1 parkour sim --domain 1
./g1 hybrid sim --domain 1

# 实机仅在完成硬件准备后使用，替换为实际网卡：
./g1 dwaq real eth0
./g1 parkour real eth0
./g1 hybrid real eth0
```

实机使用 DDS 域 `0` 和连接控制板的网卡，仿真使用正数域和 `lo`，两端域号必须一致。`./g1 run eth0` 等旧命令仍兼容。

仓库不构建自动化测试。离线检查、空跑和仿真不能替代实机运动验收。

## door 仿真快速启动

终端一：

```bash
.venv/bin/python apps/simulation/sim.py \
  --mujoco-root ~/Projects/Unitree/unitree_mujoco_sim \
  --profile door --follow
```

终端二：

```bash
./g1 dwaq sim --domain 1 --profile door
```

door 当前只使用 DWaQ 进行仿真移动，并显示独立 RGB/Depth 双视角。未来将用分离上下肢或统一全身策略替换当前 DWaQ；本阶段不实现门任务状态机或开门决策。

地形、相机管线、手柄和其他启动选项见 [仿真说明](apps/simulation/README.md)。

## 实机只读检查与空跑

```bash
./g1 interfaces
./g1 read-only eth0
./g1 joystick
```

只读模式不加载策略、不创建 LowCmd 发布器、不释放官方运动服务。

控制器默认空跑。实机真实输出需要显式 `Start/9` 接管，并在接管后重新复位；停止时先发送阻尼命令。首次实机测试必须使用吊架和独立停止手段，具体要求见 [硬件控制说明](docs/HARDWARE.md)。

## 任务档案与输入

任务档案按“程序/策略＋场景＋机器人＋相机”组合模块：

```bash
./g1 dwaq sim --domain 1 --profile door
./g1 parkour sim --domain 1 --profile parkour
./g1 hybrid sim --domain 1 --profile hybrid
```

不带 `--profile` 时使用各程序原有默认配置。字段和兼容规则见 [任务档案说明](profiles/README.md)。

仿真启动时自动识别唯一兼容手柄，无需设置 `--joystick`；也可用该参数显式指定设备。没有手柄时使用键盘，多个兼容手柄时要求明确选择。自动识别只在启动时执行，不支持热插拔重连；实机只使用原配遥控器。

## 目录

```text
apps/        各控制器、仿真、RealSense 和只读网页工具
common/      公共控制与推理模块
profiles/    任务组合配置
cameras/     相机流、视场和外参
robots/      机器人结构与安装配置
docs/        硬件、网页、部署和 Parkour 专项说明
scripts/     ARM64 机器人环境配置
cmake/       构建信息模板
build/       本机构建产物
install/     可部署安装目录
```

源码入口固定为 `src/main.cpp`（入口）、`src/options.cpp`（参数）和 `src/controller.cpp`（控制循环）。公共实现放在 `common/`，不在任务间复制。

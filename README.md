# g1_deploy — G1 机载部署

面向宇树 G1 29 自由度人形机器人的 C++17 部署项目。直接通过 Unitree DDS 读取低层状态并下发关节目标，使用 ONNX Runtime 执行预训练策略。

核心控制链路不依赖 ROS、Python 或 MuJoCo。Python 用于仿真、只读网页监控、独立相机预览和头部标定。当前包含三个行走控制器（DWaQ、Parkour、Hybrid）、任务档案组合机制和 MuJoCo 仿真启动器（含 door 任务场景与 RGB/Depth 双视角预览）。

## 安装与构建

选择一种构建方式，在源码根目录执行。两种方式都将程序安装到 `install/`。

### 自动构建

适用于 G1 的机载电脑 PC2（Jetson ARM64、Ubuntu 20.04、glibc 2.31）。[`setup_robot.sh`](scripts/setup_robot.sh) 会准备 C++ 和网页监控依赖，构建项目并运行三套策略的离线检查。

使用与源码配套的依赖包和清单。仓库默认清单留空，请将随包的 `robot-deps-release.json` 放到 `scripts/`。离线安装使用包的实际路径，在线安装需要清单中另有 HTTPS 下载地址。详见[新机部署指南](docs/ROBOT_SETUP.md)。

```bash
# 离线，从本地依赖包安装
bash scripts/setup_robot.sh --offline /home/unitree/robot-deps-focal-arm64-v1.tar.gz --install-system-deps

# 在线，从清单指定的地址下载依赖包
bash scripts/setup_robot.sh --online --install-system-deps
```

`--install-system-deps` 仅在缺少系统包时从包内补装，无需先执行 `apt-get update`。已有脚本准备的 `.deps/` 时，可用 `bash scripts/setup_robot.sh --build-only` 重新构建。

### 手动构建

适用于依赖已就绪的 Linux 开发机或 PC2。需要 CMake ≥ 3.16、C++17 编译器，以及与目标架构匹配的 Unitree SDK2、Boost、yaml-cpp、OpenCV 和 CPU 版 ONNX Runtime。将示例中的 ONNX Runtime 路径换成包含 `include/` 和 `lib/` 的实际目录。

```bash
uname -m
export ONNXRUNTIME_ROOT=/path/to/native/onnxruntime
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
```

CMake 会自动检测 RealSense SDK。实机相机可用 `-DG1_REQUIRE_REALSENSE=ON` 要求构建时找到该 SDK。必要时用 `-Dunitree_sdk2_DIR=<目录>` 或 `-Drealsense2_DIR=<目录>` 指定 SDK 的 CMake 配置目录。

`install/` 随附 ONNX Runtime。运行时还需目标系统上的 SDK/CycloneDDS 等动态库，网页监控需另行准备 [Python 依赖](docs/WEB.md#运行依赖)。

### 离线自检

以下检查不连接 DDS，也不需要机器人或相机：

```bash
ldd install/bin/g1_dwaq       # 不应出现 not found
./install/g1 dwaq check
./install/g1 parkour check
./install/g1 hybrid check
```

本项目使用上述离线命令和实机只读诊断做自检，目前没有注册 CTest 测试。如需打包当前架构的安装产物，可执行 `(cd build && cpack -G TGZ)`。

## 任务启动

启动器按任务档案选择策略、场景、机器人和相机。以下以 `door` 为例，任务名可换成 `dwaq`、`parkour` 或 `hybrid`。详见 [任务档案说明](profiles/README.md)。

### 实机

#### 网卡配置与只读检查

实机任务和监控可用源码根目录的 `./g1` 或安装后的 `./install/g1` 启动。先在 PC2 的源码根目录执行：

```bash
cp -n .g1-onboard.conf.example .g1-onboard.conf
./g1 interfaces                 # 查看本机网卡及到 PC1 的路由
```

在 `.g1-onboard.conf` 中取消 `ROBOT_INTERFACE` 的注释并填入通往 PC1 的实际网卡。实机使用 DDS 域 `0`。将下面命令中的 `eth0` 换成该网卡，检查 LowState、CRC、电机和 IMU：

```bash
./g1 read-only eth0
```

只读检查仅订阅机器人状态，可在原厂运动服务运行时检查 DDS 链路。

#### 网页监控与相机

可在 PC2 单独运行监控，或保持监控运行，再在另一个终端启动控制器。

```bash
./g1 monitor --profile door    # 关节、IMU、任务配置的 D435 深度和录制
```

直接运行 `./g1 monitor` 时，D435 深度默认使用 640×480 @30 Hz。要检查点云是否以该速率实际保存，请按 [实机帧率诊断](apps/monitor/README.md#实机帧率诊断) 核对录制结果。

监控和独立 RealSense 预览默认只监听 PC2 本机，端口分别是 `8765` 和 `8766`。从工控机访问时，建议使用 [SSH 端口转发](docs/WEB.md#从工控机访问)。若在可信局域网中需要直连，可以在 PC2 上选择运行以下命令之一：

```bash
./g1 monitor --host 192.168.123.164
./g1 realsense --host 192.168.123.164
```

工控机浏览器分别打开 [监控网页](http://192.168.123.164:8765/) 或 [相机预览](http://192.168.123.164:8766/)。`--host` 必须是已分配给 PC2、且工控机可达的 IP，不是工控机自身的 IP。`192.168.123.164` 是 PC2 的默认 IP；如果网络配置不同，请换成实际地址。只有需要监听所有网卡时才用 `--host 0.0.0.0`；只有需要更改端口时才加 `--port`。

如果监控总是使用同一个地址，可在 `.g1-onboard.conf` 中设置 `MONITOR_HOST=192.168.123.164`，之后直接运行 `./g1 monitor`；命令行的 `--host` 优先。

D435 同一时刻只能打开一路深度采集。Parkour 或 Hybrid 占用相机时，用 `./g1 monitor --joints-only` 查看机器人状态。需要独立检查原生深度时，先退出其他相机采集程序，再运行 `./install/bin/g1_camera_check`。详见[相机模块说明](profiles/configs/cameras/README.md)。

#### 控制器空跑

在 PC2 源码根目录启动任务档案指定的控制器：

```bash
./g1 task door real
```

程序启动时处于空跑。用原配遥控器按 `X`，等复位完成后按 `A`，可在监控中查看策略目标。`Y` 清零速度，`B` 或 `Select` 停止并退出。控制器终端也可用空格或 Ctrl+C 停止。

`Start` 或键盘 `9` 会申请实机接管。首次实机运动测试须使用吊架和独立停止手段。接管后须重新复位，正常停止时控制器会先发送阻尼命令。离线检查、空跑和仿真结果仅覆盖各自的检查范围，实机运动须现场验收。详见[硬件控制说明](docs/HARDWARE.md)。

### 仿真

先完成 C++ 构建，并在场景使用的 Python 环境中装好 MuJoCo、NumPy、OpenCV、Unitree Python SDK 及其 CycloneDDS 依赖。启动器优先使用 `PYTHON_BIN`，其次是源码目录的 `.venv/bin/python`，最后才是系统 `python3`。在工控机上以同一用户开两个终端，从源码目录分别启动场景和策略。同一时间运行一套仿真。仿真器内置于 `third_party/unitree_mujoco`，可用 `--mujoco-root` 指向其他等价检出路径。

```bash
./g1 task door sim scene       # 终端一，启动 MuJoCo
./g1 task door sim policy      # 终端二，启动档案指定的策略
```

两端默认使用 `lo` 和 DDS 域 `1`。如用 `--domain` 更换域号，两个终端须使用同一个正数域号。场景命令可加 `--follow` 跟随机器人。

`door` 档案加载门场景、后仰 20° 的头部和仿真相机，由 DWaQ 提供行走控制。带相机的档案在 `http://127.0.0.1:8765/` 提供监控和录制：`door` 网页显示深度，`parkour` 和 `hybrid` 网页显示原始深度和模型输入。`dwaq` 档案没有配置相机网页。录制输出到 `logs/recordings/`，保存逐帧 PLY 点云、时间戳、状态和元数据，不保存 RGB 或 MP4。详见 [仿真说明](apps/simulation/README.md) 和 [录制格式](apps/monitor/README.md#录制格式)。

以 DWaQ 为例，按 `9/Start` 启用输出，再按 `p/X` 复位，约 3 秒后按 `r/A` 进入策略。键盘用 `w/s/a/d/q/e` 移动或转向，`x` 清零，空格停止。

仿真启动时自动识别唯一兼容手柄。没有手柄时使用键盘，有多个兼容手柄时在策略命令后加 `--joystick <设备>`。自动识别只在启动时执行，不支持热插拔重连。可用 `./g1 joystick` 诊断手柄输入。

仿真中按手柄 `Select`（Xbox 的 Back/View 键），或在 MuJoCo 窗口按 `V`，可依次切换跟随、机身后方和第一人称视角。只启动场景也可以切换。

### 兼容命令

已有脚本仍可使用 `./g1 dwaq sim --profile door`、`./g1 dwaq real eth0`、`./g1 run eth0` 和 `./g1 monitor eth0 [HOST] [PORT]` 等显式命令。需要直接指定混合地形（`--terrain mixed`）时，可沿用独立仿真入口。详见 [仿真选项](apps/simulation/README.md#启动选项)。

## 目录结构

```text
apps/                       控制器、仿真、相机工具和只读网页工具
  policies/                 DWaQ、Hybrid 和 Parkour 控制器
common/                     公共控制与推理模块（header-only）
profiles/                   任务档案：组合策略程序、场景、机器人与相机
  configs/cameras/          相机流格式、视场和外参
  configs/robots/           机器人标准头部与后仰安装配置
docs/                       硬件、网页、部署和 Parkour 专项说明
scripts/                    ARM64 机器人环境配置
cmake/                      构建信息模板
build/                      本机构建产物
install/                    可部署安装目录
```

每个 C++ 控制器的源码入口固定为 `src/main.cpp`（入口）、`src/options.cpp`（参数解析）和 `src/controller.cpp`（控制循环）。跨程序实现放在 `common/`，任务在 `profiles/` 中组合。各程序的 JSON 配置在启动时读取，修改后须重启对应程序。

## 模块与文档导航

| 模块 | 功能 | 专题文档 |
| --- | --- | --- |
| DWaQ | 单网络行走、复位、离线检查和手柄诊断 | [apps/policies/dwaq/README.md](apps/policies/dwaq/README.md) |
| Parkour | 深度编码器＋动作网络双 ONNX 推理 | [apps/policies/parkour/README.md](apps/policies/parkour/README.md) |
| Hybrid | DWaQ 与 Parkour 按前进速度切换 | [apps/policies/hybrid/README.md](apps/policies/hybrid/README.md) |
| 仿真 | 随仓库提供的仿真适配与场景资产，运行时需安装 MuJoCo | [apps/simulation/README.md](apps/simulation/README.md) |
| 头部标定 | 利用双手轮廓求解头部后仰角、旋转轴和相机外参（从源码运行） | [apps/head_calibration/README.md](apps/head_calibration/README.md) |
| RealSense | C++ 深度采集、相机检查和独立网页预览 | [apps/realsense/README.md](apps/realsense/README.md) |
| 数据监控 | 关节、IMU、空跑目标、实时深度和点云录制 | [apps/monitor/README.md](apps/monitor/README.md) |
| 任务档案 | 组合程序/策略、场景、机器人和相机 | [profiles/README.md](profiles/README.md) |
| 相机模块 | 流格式、视场、RGB–Depth 外参和官方参数 | [profiles/configs/cameras/README.md](profiles/configs/cameras/README.md) |
| 机器人模块 | 标准头部和后仰安装配置 | [profiles/configs/robots/README.md](profiles/configs/robots/README.md) |
| 公共控制底座 | 控制循环、DDS、推理线程、输入与硬件检查 | [common/README.md](common/README.md) |

补充文档：

- [硬件控制与验证边界](docs/HARDWARE.md)
- [Parkour 与 RealSense 详细说明](docs/PARKOUR.md)
- [网页监控部署](docs/WEB.md)
- [远端观测与目标跟随（仿真 / 实机空跑）](docs/REMOTE_CONTROL.md)
- [ARM64 机器人一键配置](docs/ROBOT_SETUP.md)
- [模型与资产来源](ORIGIN.md)
- [历史修改与验证记录](CHANGELOG.md)

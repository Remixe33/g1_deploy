# g1_deploy — G1 机载部署

**部署到新 G1：** 源码 ZIP 不含机载依赖。安装脚本支持在线下载和离线包，两种方式都不需要旧部署。当前源码尚未配套发布 ARM64 依赖包，不能直接用 `--online` 或 `--offline` 完成新机安装；制包与验收步骤见 [新机部署指南](docs/ROBOT_SETUP.md)。

## 简要介绍

面向宇树 G1 29 自由度人形机器人的 C++17 部署项目。直接通过 Unitree DDS 读取低层状态并下发关节目标，使用 ONNX Runtime 执行预训练策略。

核心控制链路不依赖 ROS、Python 或 MuJoCo。Python 用于仿真、只读网页监控、独立相机预览和头部标定。当前包含三个行走控制器（DWaQ、Parkour、Hybrid）、任务档案组合机制和 MuJoCo 仿真启动器（含 door 任务场景与 RGB/Depth 双视角预览）。

### 模块与文档导航

| 模块 | 功能 | 专题文档 |
| --- | --- | --- |
| DWaQ | 单网络行走、复位、离线检查和手柄诊断 | [apps/policies/dwaq/README.md](apps/policies/dwaq/README.md) |
| Parkour | 深度编码器＋动作网络双 ONNX 推理 | [apps/policies/parkour/README.md](apps/policies/parkour/README.md) |
| Hybrid | DWaQ 与 Parkour 按前进速度切换 | [apps/policies/hybrid/README.md](apps/policies/hybrid/README.md) |
| 仿真 | 随仓库提供的仿真适配与场景资产，运行时需安装 MuJoCo | [apps/simulation/README.md](apps/simulation/README.md) |
| 头部标定 | 利用双手轮廓求解头部后仰角、旋转轴和相机外参（从源码运行） | [apps/head_calibration/README.md](apps/head_calibration/README.md) |
| RealSense | C++ 深度采集、相机检查和独立网页预览 | [apps/realsense/README.md](apps/realsense/README.md) |
| 数据监控 | 关节、IMU、空跑目标、RGB-D 预览和录制 | [apps/monitor/README.md](apps/monitor/README.md) |
| 任务档案 | 组合程序/策略、场景、机器人和相机 | [profiles/README.md](profiles/README.md) |
| 相机模块 | 流格式、视场、RGB–Depth 外参和官方参数 | [profiles/configs/cameras/README.md](profiles/configs/cameras/README.md) |
| 机器人模块 | 标准头部和后仰安装配置 | [profiles/configs/robots/README.md](profiles/configs/robots/README.md) |
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

每个 C++ 控制器的源码入口固定为 `src/main.cpp`（入口）、`src/options.cpp`（参数解析）和 `src/controller.cpp`（控制循环）。跨程序实现放在 `common/`，不在任务间复制。任务与模块解耦：模块文件按自身命名，任务组合只发生在 `profiles/`。

## 手工构建

以下命令适用于依赖已就绪的 Linux 环境，从源码根目录执行。新机部署流程见 [新机部署指南](docs/ROBOT_SETUP.md)。

依赖：Linux、CMake≥3.16、C++17、Unitree SDK2、Boost、yaml-cpp、OpenCV 和 CPU 版 ONNX Runtime。RealSense C++ SDK 自动检测；实机相机构建建议加 `-DG1_REQUIRE_REALSENSE=ON`。

SDK、ONNX Runtime 和库文件必须与目标 CPU 架构匹配。将示例中的 ONNX Runtime 路径换成实际安装目录（应包含 `include/` 和 `lib/`）。如果 CMake 找不到已安装的 Unitree SDK2 或 RealSense SDK，可分别用 `-Dunitree_sdk2_DIR=<目录>`、`-Drealsense2_DIR=<目录>` 指定其 CMake 配置所在目录。

```bash
uname -m
export ONNXRUNTIME_ROOT=/path/to/native/onnxruntime
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DONNXRUNTIME_ROOT="$ONNXRUNTIME_ROOT" \
  -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j4
cmake --install build
```

实机任务和监控可从源码根目录或安装目录启动。安装目录自带 ONNX Runtime，但仍依赖目标系统上的 SDK/CycloneDDS 和其他动态库；网页监控另需 [Python 运行依赖](docs/WEB.md#运行依赖)。

### 离线自检

以下检查不连接 DDS，也不需要机器人或相机：

```bash
ldd install/bin/g1_dwaq       # 不应出现 not found
./install/g1 dwaq check
./install/g1 parkour check
./install/g1 hybrid check
```

需要当前架构的安装目录压缩包时，可从源码根目录运行以下命令。它不是新机安装所需的离线依赖包：

```bash
cd build
cpack -G TGZ
```

## 任务启动

启动器按任务档案选择策略、场景、机器人和相机。以下以 `door` 为例，任务名可换成 `dwaq`、`parkour` 或 `hybrid`。详见 [任务档案说明](profiles/README.md)。

### 仿真

先完成 C++ 构建，并在场景使用的 Python 环境中装好 MuJoCo、NumPy、OpenCV、Unitree Python SDK 及其 CycloneDDS 依赖。启动器优先使用 `PYTHON_BIN`，其次是源码目录的 `.venv/bin/python`，最后才是系统 `python3`。在工控机上以同一用户开两个终端，从源码目录分别启动场景和策略。同一时间运行一套仿真。

```bash
./g1 task door sim scene       # 终端一，启动 MuJoCo
./g1 task door sim policy      # 终端二，启动档案指定的策略
```

两端默认使用 `lo` 和 DDS 域 `1`。如用 `--domain` 更换域号，两个终端须使用同一个正数域号。场景命令可加 `--follow` 跟随机器人。

`door` 档案加载门场景、后仰 20° 的头部和 RGB-D 相机，由 DWaQ 提供行走控制。带相机的档案在 `http://127.0.0.1:8765/` 提供监控和录制。`door` 显示 RGB 和深度，`parkour`、`hybrid` 显示原始深度和模型输入。`dwaq` 档案没有配置相机网页。录制输出到 `logs/recordings/`，`door` 录制包括 RGB 与深度伪彩 MP4、PLY 点云、状态 CSV 和元数据。详见 [仿真说明](apps/simulation/README.md) 和 [录制格式](apps/monitor/README.md#录制格式)。

以 DWaQ 为例，按 `9/Start` 启用输出，再按 `p/X` 复位，约 3 秒后按 `r/A` 进入策略。键盘用 `w/s/a/d/q/e` 移动或转向，`x` 清零，空格停止。

仿真启动时自动识别唯一兼容手柄。没有手柄时使用键盘，有多个兼容手柄时在策略命令后加 `--joystick <设备>`。自动识别只在启动时执行，不支持热插拔重连。

仿真中按手柄 `Select`（Xbox 的 Back/View 键），或在 MuJoCo 窗口按 `V`，可依次切换跟随、机身后方和第一人称视角。只启动场景也可以切换。

### 实机

在 PC2 将 `.g1-onboard.conf.example` 复制为 `.g1-onboard.conf`。用 `./g1 interfaces` 查看通往 PC1 的 DDS 网卡，取消配置文件中 `ROBOT_INTERFACE` 的注释并填入实际网卡名。随后从工控机开两个 SSH 终端登录 PC2，分别运行：

```bash
./g1 task door real            # 终端一，启动档案指定的策略，初始为空跑
./g1 monitor                   # 终端二，关节、IMU、D435 深度和录制
```

监控和独立 RealSense 预览默认只监听 PC2 本机，端口分别是 `8765` 和 `8766`。从工控机访问时，建议使用 [SSH 端口转发](docs/WEB.md#从工控机访问)。若在可信局域网中需要直连，可以在 PC2 上选择运行以下命令之一：

```bash
./g1 monitor --host 192.168.123.164
./g1 realsense --host 192.168.123.164
```

工控机浏览器分别打开 [监控网页](http://192.168.123.164:8765/) 或 [相机预览](http://192.168.123.164:8766/)。`--host` 必须是已分配给 PC2、且工控机可达的 IP，不是工控机自身的 IP。`192.168.123.164` 是 PC2 的默认 IP；如果网络配置不同，请换成实际地址。只有需要监听所有网卡时才用 `--host 0.0.0.0`；只有需要更改端口时才加 `--port`。

如果监控总是使用同一个地址，可在 `.g1-onboard.conf` 中设置 `MONITOR_HOST=192.168.123.164`，之后直接运行 `./g1 monitor`；命令行的 `--host` 优先。

监控可独立于策略运行。Parkour 和 Hybrid 占用 D435 时，用 `./g1 monitor --joints-only` 查看机器人状态。详见 [实机部署](docs/ROBOT_SETUP.md)。

仿真 USB 手柄和实机原配遥控器使用相同按键。按 `Start` 启用输出、`X` 复位、`A` 进入策略、`Y` 清零速度、`B` 停止并退出。实机也可按 `Select` 停止。控制器终端可用空格或 Ctrl+C 停止。

实机空跑时直接按 `X`，等复位完成后按 `A`。启用真实输出后须重新复位。接管后正常停止时，控制器会先发送阻尼命令。

### 诊断与兼容命令

```bash
./g1 interfaces        # 列出网卡与到 PC1 的路由
./g1 read-only eth0    # 将 eth0 换成实际 DDS 网卡，检查 LowState、CRC、电机和 IMU
./g1 monitor           # 只读网页监控：关节、IMU 和 D435 深度；不采集相机时加 --joints-only
./g1 joystick          # 手柄输入诊断
./g1 realsense         # 独立 RealSense 网页预览，默认仅本机；局域网直连需显式加 --host
./install/bin/g1_camera_check  # 需启用 RealSense SDK 并连接空闲的 D435，仅检查深度
```

只读模式不加载策略、不创建 LowCmd 发布器、不释放官方运动服务。

已有脚本仍可使用 `./g1 dwaq sim --profile door`、`./g1 dwaq real eth0`、`./g1 run eth0` 和 `./g1 monitor eth0 [HOST] [PORT]` 等显式命令。需要直接指定混合地形（`--terrain mixed`）时，可沿用独立仿真入口。详见 [仿真选项](apps/simulation/README.md#启动选项)。

## 注意事项

- 首次实机测试必须使用吊架和独立停止手段，实机相关边界见 [硬件控制说明](docs/HARDWARE.md)。
- 本项目未注册 CTest 测试；请使用上面的离线自检与只读诊断。离线检查、空跑和仿真不能替代实机运动验收。
- 实机使用 DDS 域 `0` 和连接控制板的网卡，仿真使用正数域和 `lo`，两端域号必须一致。
- 各程序 JSON 配置仅在启动时读取，修改后须重启生效。
- D435 单深度传感器同一时刻只能开一路流，原生深度流与 Parkour 策略管线在实机互斥（详见 [相机模块说明](profiles/configs/cameras/README.md)）。
- 仿真器已内置于 `third_party/unitree_mujoco`（来源见 [ORIGIN.md](ORIGIN.md)）；`--mujoco-root` 可指向其他等价检出路径。

# 一键配置机器人（手动复制文件）

适用当前 G1 开发电脑：Linux ARM64/aarch64，原依赖流程曾在 Ubuntu 20.04 验证；新增的原生相机需单独验证。不会自动 SSH/SCP/rsync，不启动运动控制，不改系统时间或系统 glibc。

## 使用

1. 手动把 `D:\g1_deploy` 复制到机器人，例如 `/home/unitree/g1_deploy`。同时复制 `.deploy-cache/robot-arm64/`，离线依赖约 100 MB；不要复制 Windows 的 `build/`、`install/` 或另一目录的 `.deps/`。
2. 在机器人上执行：

```bash
cd ~/g1_deploy
bash scripts/setup_robot.sh
```

有缺失的 Ubuntu 编译依赖时可使用下面的命令；只有确实缺少软件包时才调用 sudo/apt，需要网络和管理员密码：

```bash
bash scripts/setup_robot.sh --install-system-deps
```

额外验证 C++ 原生相机出图（只读，不启动控制程序）：

```bash
bash scripts/setup_robot.sh --verify-camera
```

仅配置依赖，不编译控制程序：

```bash
bash scripts/setup_robot.sh --deps-only
```

安装后，在 PC2 的部署根目录（例如 `~/g1_deploy`）配置一次机内 DDS 网卡：

```bash
cp -n .g1-onboard.conf.example .g1-onboard.conf
./g1 interfaces
```

编辑 `.g1-onboard.conf`，取消 `ROBOT_INTERFACE` 这一行的注释，并填入通往 PC1 的实际网卡名。工控机访问网页使用 PC2 的 Wi-Fi 地址。从源码目录和其下的 `install/` 启动时，共用这一份本机配置。独立解压的安装包在自身根目录配置。

随后在两个 SSH 终端中分别运行：

```bash
./g1 task door real            # 终端一，启动 DWaQ，初始为空跑
./g1 monitor                   # 终端二，关节、IMU、D435 RGB-D 和录制
```

浏览器打开 `http://<PC2的Wi-Fi-IP>:8765/`。监控服务可单独运行，通过网页按钮开始或停止录制，数据保存到本机配置所在目录的 `logs/recordings/`。可在配置中用 `MONITOR_HOST`、`MONITOR_PORT` 调整默认的 `0.0.0.0:8765`。

`./g1 task parkour real` 和 `./g1 task hybrid real` 会在策略进程内打开 D435。运行期间用 `./g1 monitor --joints-only` 查看机器人状态。需要独立 RGB-D 预览或录制时，先退出占用相机的策略。

原配遥控器按键与仿真 USB 手柄一致：Start 启用输出、X 复位、A 进入策略、Y 清零速度、B 停止退出。实机也可按 Select 停止。
空跑用 X → 等待复位完成 → A。真实控制需要现场保护，Start → X → 等待复位完成 → 摇杆回中 → A。
配置已将左右 hip yaw 限制为 ±90°，其它关节保持原配置。

## 原生相机依赖

机器人构建强制要求 `librealsense2` C++ 开发库，配置脚本设置 `G1_REQUIRE_REALSENSE=ON`。SDK 可安装到系统，或放在 `.deps/realsense` 并包含 CMake 配置、头文件及本机 ABI 兼容的库。旧离线缓存中的 `pyrealsense2` wheel 和私有 glibc 仅服务于独立 Python 预览，不满足 C++ 依赖；离线迁移须额外准备原生 SDK。`--install-system-deps` 会尝试安装 `librealsense2-dev`，目标软件源须提供该包。

实机启动 `./g1 parkour real eth0` 或 `./g1 hybrid real eth0` 即可，先关闭任何占用相机的独立预览。只读相机诊断使用 `./bin/g1_camera_check`，不需要网页端口或 Python 相机服务。

## 配置内容

- 项目 `.deps/runtime`：Python 3.10.15；`.deps/python`：独立虚拟环境。
- NumPy 1.26.4、CycloneDDS Python 0.10.2、宇树 Python SDK、OpenCV headless 4.11.0.86、RealSense Python 2.55.1.6486。
- `.deps/unitree-sdk2`、`.deps/cyclonedds`、`.deps/onnxruntime`：已在此机器人验证的 ARM64 原生库；构建不再依赖其它项目目录。
- `.deps/glibc-2.35`：Ubuntu 官方 ARM64 glibc 2.35。只有相机专用 Python 入口通过它的动态加载器启动；不修改 `/lib`、`/usr/lib`、系统加载器或全局环境变量。
- `apps/realsense/python3-realsense.sh`：相机专用入口模板，由配置脚本复制到 `.deps/python/bin/python3-realsense`。RGB-D 监控和独立网页预览可使用这个入口。纯关节监控和控制程序沿用各自的常规运行库。
- 编译 Release、安装到 `install/`，并完成两套模型的离线推理检查。已有安装会备份到 `deployment-backups/`，发现本项目控制程序正在运行则停止配置，需先由操作者退出程序。
- 日志及包清单位于 `.deps/setup-logs/`。

## 离线缓存与迁移

`.deploy-cache/robot-arm64/` 当前已在本地准备齐全，包含 Python、SDK 和运行库归档、Python wheels、glibc deb，以及 SHA256SUMS。配置脚本先校验完整性，再离线安装。可用 `--cache /path/to/robot-arm64` 指定其它位置。

缓存来自官方发行包及当前机器人已验证的原生库。该缓存仅适用于 ARM64，基础系统需有兼容的 C++/OpenCV/yaml-cpp/Boost 依赖；其它架构需另备依赖包。

`.deps/`、`.deploy-cache/`、`build-sdk/` 和安装/构建目录已加入 Git 忽略。Git 保存配置流程和启动脚本，不保存平台二进制及本机虚拟环境。若使用 Git 将源码带到另一台机器，需要另外手动复制缓存；仅复制源码不足以离线配置。不得把旧目录的虚拟环境整体移动后直接使用，应在目标目录重新运行配置脚本。

此脚本不设置开机自启，也不执行电机接管。Parkour 和 Hybrid 默认使用 480×270、60 Hz 深度。独立 RGB-D 监控默认使用 640×480、30 Hz。实际采集频率由页面和日志统计。

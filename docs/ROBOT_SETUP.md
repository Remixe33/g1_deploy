# G1 新机部署

首版依赖包的目标是 Jetson aarch64、Ubuntu 20.04（反馈机为 20.04.6）和 glibc 2.31。系统自带的 Python 3 用于安装脚本；监控使用包内的 Python 3.10.15，不改系统 Python、glibc、JetPack 或内核。部署不启动控制器，也不发送运动指令。

## 1. 解压源码，选择安装方式

源码 ZIP **不含依赖**。使用同一次发布中的源码与依赖包。若 ZIP 已带顶层目录，在目标父目录执行 `unzip g1_deploy-door.zip`，不要再加同名解压目录。通过 SCP 传输时，目标使用实际绝对路径，如 `unitree@<PC2地址>:/home/unitree/`，不要依赖远端 `~` 展开。

进入解压后的源码根目录，只选择一种方式：

```bash
# 在线：以源码内的清单为准，从发布地址下载固定版本的包
bash scripts/setup_robot.sh --online --install-system-deps

# 离线：把同版本依赖包传到机器人，不访问软件源或 PyPI
bash scripts/setup_robot.sh --offline /home/unitree/robot-deps-focal-arm64-v1.tar.gz --install-system-deps
```

`--install-system-deps` 授权在缺少系统软件包时通过 `sudo` 安装包内的本地 `.deb`；不传此选项则只列出缺失的包。安装器拒绝会移除软件包或修改系统 glibc、内核、JetPack 的安装方案。在线下载需要 PC2 能访问发布地址并具备有效的系统 HTTPS 证书；离线可在另一台机器下载后传入。

两条路径在解包前检查发布清单中的 SHA-256，随后检查包内文件清单和版本。校验失败时重新获取匹配的安装包，不要手动改校验值。源码开发版若显示没有发布校验值，表示对应的依赖包尚未出包，不是机器人缺库。无参数只读取本地 `.deploy-cache/robot-arm64`，不会自动联网；也可以用 `--cache DIR` 指定已解包的缓存。

只装依赖可在上述命令后加 `--deps-only`；依赖已准备好时，用 `bash scripts/setup_robot.sh --build-only` 重新构建并安装，无需缓存或联网。安装到 `install/` 后会检查 DWaQ、Parkour 和 Hybrid 离线推理，日志与 Python 包清单位于 `.deps/setup-logs/`。

## 2. 首次配置与只读自检

```bash
cp -n .g1-onboard.conf.example .g1-onboard.conf
./g1 interfaces
```

在 `.g1-onboard.conf` 里取消 `ROBOT_INTERFACE` 的注释并填入 PC2 通往 PC1 的实际网卡。从源码或其下的 `install/` 启动会共用这份配置。使用显式网卡也可以启动只读检查或监控：

```bash
./install/g1 dwaq check
./install/g1 read-only eth0    # 用实际 DDS 网卡替换 eth0
./install/bin/g1_camera_check # 验证 C++ 深度，先退出其它相机采集程序
./install/g1 monitor
```

监控默认显示关节、IMU 和 D435 深度（640×480 @30 Hz）。`./install/g1 monitor --profile door` 会读取 Door 任务的相机配置，当前也是 640×480 @30 Hz。默认只监听 PC2 的 `127.0.0.1:8765`。工控机可用 [SSH 端口转发](WEB.md#从工控机访问) 访问；可信网络中也可显式传入 `--host 0.0.0.0`，再打开日志中可达的 PC2 地址。已有配置中的 `MONITOR_HOST`、`MONITOR_PORT` 仍然生效，如需恢复仅本机访问，请移除原有的 `MONITOR_HOST` 或改为 `127.0.0.1`。录制保存在 `logs/recordings/`。

`./g1 monitor --interface eth0 --joints-only` 不开相机，适用于 Parkour 或 Hybrid 已占用 D435 的场景。Monitor 只开启深度流，录制保留点云和时间戳，不保存 RGB 或 MP4。无相机时可先用 `--joints-only` 验证网页和 DDS。更多细节见 [网页监控](WEB.md) 和 [RealSense 说明](../apps/realsense/README.md)。

## 发布依赖包

在与目标一致的干净 ARM64/Focal 用户空间上制作。先在未安装项目软件包的目标镜像上配置与原厂镜像匹配的 APT 源，更新索引，再运行 `bash scripts/package_robot_deps.sh collect-debs /path/to/inputs/deb`。它按 [系统包清单](../scripts/robot-system-packages.txt) 解析并下载缺失包和传递依赖。如果解析要求升级系统 glibc、内核或 JetPack，先校正构建镜像与 APT 源，不绕过检查。

在构建环境准备以下 `inputs/`，每个 SDK 目录是从官方源码或兼容发行包重新准备的安装前缀，不从旧机器的 `.deps` 复制。打包脚本会在这个系统上检查动态库，并用临时 venv 验证完整 wheel 集可离线安装和导入：

- `runtime/python/`：CPython 3.10.15（启用 `--enable-shared --with-ensurepip=install`，验证 `ssl` 与 `venv`）。
- `unitree-sdk2/`、`cyclonedds/`、`onnxruntime/`：分别安装 G1 兼容的 [Unitree SDK2](https://github.com/unitreerobotics/unitree_sdk2)、[CycloneDDS 0.10.2](https://github.com/eclipse-cyclonedds/cyclonedds/tree/0.10.2) 和 CPU 版 [ONNX Runtime 1.18.1](https://github.com/microsoft/onnxruntime/tree/v1.18.1)。安装库到 `lib/`，同时保留头文件、CMake 配置、SONAME 软链和上游许可证；如果预编译库要求高于 glibc 2.31 的符号，须在目标用户空间从对应源码重建。
- `realsense/` 与 `wheels/pyrealsense2-*.whl`：从 [librealsense v2.55.1](https://github.com/realsenseai/librealsense/tree/v2.55.1) 构建。在构建机装好 CMake、libusb/udev 开发头文件和 Python 的 `wheel` 后，执行 `bash scripts/build_robot_realsense.sh /path/to/librealsense /path/to/build-venv/bin/python /path/to/inputs`。构建脚本验证原生 SDK 与 Python 绑定能否在普通 Python 中导入，不启用私有 glibc。
- `wheels/`：使用相同的 Python 3.10，在目标架构上准备 NumPy 1.26.4、OpenCV 4.11.0.86（`opencv-python`）、CycloneDDS 0.10.2、`unitree_sdk2py` 1.0.1 及所有传递依赖的 wheel。CycloneDDS 如需从源码构建，设置 `CYCLONEDDS_HOME` 指向上述原生库；用 `pip wheel` 构建 SDK wheel，用 `pip download --only-binary=:all:` 收集其余依赖。每个发行包只放一个兼容版本，安装器不在线编译或解析下载。
- `SOURCES.txt`：记录镜像和 APT 来源、SDK 的发行版或 Git 提交、wheel 版本与来源、构建选项及许可证；不要分发未获再分发许可的组件。

```bash
bash scripts/package_robot_deps.sh pack /path/to/inputs /path/to/output \
  'https://<发布主机>/<固定版本路径>/robot-deps-focal-arm64-v1.tar.gz'
```

只制作离线包时可省略末尾的 HTTPS 地址，生成的发布清单仍包含校验值。将生成的 `robot-deps-release.json` 随匹配的源码放到 `scripts/`，依赖包作为单独的附件。不要提交本机的 `.deps/`、缓存、`build/` 或 `install/`。更换依赖包内容时更新版本，不覆盖已发布的对象。

在线发布前，用两份全新的目标环境分别验证在线和断网安装，并检查 Python 导入和三套策略离线推理。仓库默认清单中的下载地址和校验值留空，部署时使用与依赖包配套的清单。接上 G1 后，再独立检查 DDS 只读、深度出帧、远端网页和 RGB。

## 旧机迁移（可选）

优先解到新目录。旧布局的本地缓存可用于原有部署，不是新机依赖包；不要搬旧 venv，不要混用新旧 `.deps`。覆盖当前 `install/` 前，安装器会比较将被替换的任务档案和策略配置，并在发现差异时停止。先核对并合并需要保留的配置，尤其是策略配置中实际生效的左右髋 yaw 限位；备份并移开旧安装后再部署。还在运行的本项目控制器应由操作者先退出。

如果 SSH 登录被机载系统的 ROS 选择提示卡住，让镜像维护者把交互选择限制在交互式 shell，不要让 `.bashrc` 在非交互连接中执行 `read`。如果 RGB 开流崩溃，先恢复默认深度配置；重现时记录设备节点占用、所选流配置和 `PYTHONFAULTHANDLER=1` 的输出。旧私有加载器是 shell 脚本，调试原生栈应让调试器运行加载器或 Python 的实际可执行文件。

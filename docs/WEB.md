# 网页关节监控

网页及Python后端均已收进本仓库，无需另一个unitree_mujoco源码仓库。
监控只订阅DDS，不发布LowCmd、不释放运动服务，也不提供运动控制按钮。
它显示29个关节的角度、速度、反馈力矩、目标、误差和IMU，以及最近20秒的曲线。
实际目标与空跑目标分开处理；空跑时显示`OFF · 空跑`。

## 运行依赖

C++控制程序仍不依赖Python。仅网页需要Linux上的Python 3.10或更新版本、
CPU架构匹配的CycloneDDS C运行库、Python依赖，以及官方`unitree_sdk2py`包。
新机使用 [部署脚本](ROBOT_SETUP.md) 安装匹配的运行库；以下手工方式适合已有 SDK 的开发机。
SDK及CycloneDDS运行库和控制程序使用同一套，避免混用ROS环境中的另一套DDS库。

如果Python已经能导入`unitree_sdk2py`和`cyclonedds`，可直接运行。
否则，在仓库根目录安装到项目内；将下面的SDK源码路径替换为本机官方Python SDK路径：

```bash
CYCLONEDDS_HOME=/usr/local python3 -m pip install --target .python-deps -r apps/monitor/requirements.txt
python3 -m pip install --no-deps --target .python-deps /path/to/unitree_sdk2_python
```

官方Python SDK来源：https://github.com/unitreerobotics/unitree_sdk2_python 。
`--no-deps`避免安装本监控不使用的摄像头依赖，所需DDS和NumPy已在上一步安装。
`.python-deps/`是本机依赖目录，不提交Git。若使用虚拟环境，可通过`PYTHON_BIN`指定解释器。

## 启动

```bash
cd /home/unitree/g1_deploy
./g1 monitor                         # 读取 .g1-onboard.conf，关节 + 深度
./g1 monitor --interface eth0 --joints-only
```

监控默认只监听 PC2 的 `127.0.0.1:8765`。已有 `MONITOR_HOST` 配置仍生效，命令行 `--host` 可覆盖。实机通信域为 0，默认采样上限为 50 Hz。旧的 `INTERFACE [HOST] [PORT]` 位置写法仍可用，未启用相机时保持原来的纯关节行为。
按Ctrl+C停止网页，不影响其他控制程序。不设置开机自启。

也可直接启动Python后端：

```bash
python3 apps/monitor/joint_monitor.py --domain 0 --interface enP8p1s0 \
    --host 192.168.123.164 --port 8765 --rate 50
```

直接运行Python时，需要已安装依赖或自行将`.python-deps`加入`PYTHONPATH`。

## 从工控机访问

保留监控的默认监听地址，在工控机另开终端执行：

```bash
ssh -N -L 127.0.0.1:8765:127.0.0.1:8765 unitree@192.168.123.164
```

保持 SSH 连接，从工控机浏览器打开 [http://127.0.0.1:8765/](http://127.0.0.1:8765/)。若本地端口已占用，修改 `-L` 中左侧的端口和浏览器地址即可。独立 RealSense 预览可用同样的方式转发默认端口 `8766`。

如果在可信网络中需要直接访问，用 `./g1 monitor --host 0.0.0.0`（或指定一张可达网卡的 IP）启动，再打开日志中的 PC2 地址。默认机载 IP 为 `192.168.123.164`；`0.0.0.0` 不是浏览器地址。

## 安装目录

`cmake --install build`会同时安装`apps/monitor/`和`monitor`启动脚本：

```bash
cd install
./monitor enP8p1s0 192.168.123.164 8765
# 等效入口：./g1 monitor enP8p1s0 192.168.123.164 8765
```

源码与其下安装目录共用机载配置和项目 Python 环境。单独分发 CMake 安装目录时，必须另行提供 Python 依赖。
网页静态文件按Python源码位置加载，目录整体移动后仍能使用。

## 接口

- `/`：网页。
- `/api/joints`：当前29关节、IMU、数据年龄、输出和空跑状态。
- `/api/stream`：SSE实时数据流。

监控的Enable是本控制程序的输出状态，不能据此判断机器人电源或其他服务是否在控制。

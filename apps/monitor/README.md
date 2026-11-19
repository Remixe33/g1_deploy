# 只读关节网页监控

`joint_monitor.py` 显示 29 个关节、IMU、实际命令、空跑目标和最近曲线。网页只订阅 DDS，不发布 LowCmd、不释放运动服务，也不能发送运动指令。

## 启动

```bash
# 源码目录；参数依次为 DDS 网卡、HTTP 监听地址、端口
bash apps/monitor/monitor.sh eth0 0.0.0.0 8765

# 仿真直接运行，可指定域
python3 apps/monitor/joint_monitor.py \
  --domain 1 --interface lo --host 127.0.0.1 --port 8765 --rate 50

# 安装目录
./g1 monitor eth0 0.0.0.0 8765
```

浏览器打开 `http://机器人IP:8765/`；本机仿真使用 `http://127.0.0.1:8765/`。过期数据显示离线，空跑目标标为 `OFF · 空跑`。Ctrl+C 结束服务。

## 依赖

需要 Python、NumPy、CycloneDDS 和官方 `unitree_sdk2py`，版本范围见 `requirements.txt`。C++ 控制器本身不依赖这些 Python 包。

安装到项目内依赖目录的示例：

```bash
CYCLONEDDS_HOME=/usr/local python3 -m pip install \
  --target .python-deps -r apps/monitor/requirements.txt
python3 -m pip install --no-deps --target .python-deps \
  /path/to/unitree_sdk2_python
```

可通过 `PYTHON_BIN` 指定解释器。SDK 和 CycloneDDS 应与控制器使用同一套运行库，避免混用 ROS 环境中的另一套 DDS 库。

接口为 `/`、`/api/joints` 和 `/api/stream`。完整部署说明见 [网页关节监控](../../docs/WEB.md)。

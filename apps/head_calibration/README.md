# G1 双手轮廓自动标定

## 功能

用固定双手的可见轮廓、同帧 RGB 和实际关节角，求头部后仰角、躯干系轴心 X/Z 和 RGB 相机外参。操作者粗对齐并确认，程序自动求解。仿真自动摆臂，实机由现有控制方式摆臂。

## 命令

### 1. 准备工具环境

从 `g1_deploy` 根目录运行。使用 Python 3.10 或以上，仿真桌面需要可用的 OpenCV 图形窗口及 Noto Sans CJK 字体。

```bash
python3 -m venv apps/head_calibration/.venv
apps/head_calibration/.venv/bin/python -m pip install -r apps/head_calibration/requirements.txt
```

实机环境还应能导入与机载架构和 Python ABI 匹配的 `unitree_sdk2py`、CycloneDDS 和 `pyrealsense2`。JetPack 6 使用 Python 3.10 及对应的 ARM64 RealSense 绑定。可用 `G1_CALIBRATION_PYTHON` 指定已装齐依赖的解释器。[SDK 安装说明](../../docs/WEB.md#运行依赖)

### 2. 在门场景中标定

```bash
bash apps/head_calibration/calibrate.sh sim
```

等双臂稳定、青色轮廓出现。用 **A／D** 或方向键模拟调头，使双手与目标轮廓大致重合，按 **Enter** 确认。结果窗按 **Esc** 退出。可加 `--headless --score-sim --demo-back-deg 21` 运行无窗口仿真评分。

### 3. 连接实机并在浏览器标定

复制 [配置模板](real-config.example.json)，填写实机 DDS 网卡、相机序列号及经核实的零位 RGB 安装外参。机载电脑运行：

```bash
bash apps/head_calibration/calibrate.sh real --config apps/head_calibration/real-config.json
```

服务监听机载电脑的 `127.0.0.1:8768`。工作站可用 `ssh -L 8768:127.0.0.1:8768 用户@机器人地址` 建立转发，再打开 `http://127.0.0.1:8768/`。由现有控制方式摆好双臂，观察预览，调整并锁紧头轴，保持静止后确认。随后保持头轴锁紧，换一组双臂姿态并复核。

## 输出

- 默认保存在 `logs/head-calibration/` 下的独立运行目录。`--out` 可指定一个尚未创建的目录。
- 仿真的 `fit/calibration.json` 和实机的 `frame-*/fit/calibration.json` 保存通过图像质量检查的 RGB 外参。同级 `report.json` 和 `overlay.png` 保存拟合诊断与叠加图。变换方向为相机光学系到 `torso_link`，平移单位为米。
- 实机根据设备的 RGB／深度外参合成 `depth-camera-calibration.json`，其中保存深度点到躯干的变换、设备和流信息，供在线和离线深度推理显式加载。
- 仿真开启 `--score-sim` 时另存 `score.json` 与独立真值。实机在 `frame-*/` 下保存配对质量、原始及校正后的 RGB、原生深度（该帧可用时）和设备参数。留出结果见 `frame-*/check/report.json`。

## 注意事项

- 实机进程只订阅状态和读取相机。粗对齐后锁紧头轴，用现有控制方式保持双臂稳定，再确认画面。停止占用同一相机的其他采集程序后再启动。
- 拟合使用对应流的镜头模型和当前实际关节角。搬动相机或再次调头后，重新标定。实机联调和绝对误差需在设备可用时验证。
- 部署到机载电脑时，在目标机重新准备依赖，并根据配置模板创建本机的 `real-config.json`。此配置文件由 Git 忽略。

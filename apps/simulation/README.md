# MuJoCo 仿真与 door 任务

`sim.py` 按任务档案组合场景、机器人和相机，启动内置仿真器 `third_party/unitree_mujoco`（来源见 [../../ORIGIN.md](../../ORIGIN.md)）。仿真器、基础 XML 和网格随仓库提供，本项目只维护任务配置和适配层；`--mujoco-root` 可指向其他等价检出。

## 快速启动

终端一：

```bash
./g1 task door sim scene --follow
```

终端二：

```bash
./g1 task door sim policy
```

两条命令都从源码目录运行，可将 `door` 同时换成 `dwaq`、`parkour` 或 `hybrid`。场景入口依次选用 `PYTHON_BIN`、源码目录的 `.venv/bin/python` 或系统 `python3`。场景和策略分别接收各自的选项，例如 `sim scene --follow` 和 `sim policy --joystick /dev/input/js0`。

带相机的任务档案自动托管**机器人数据网页** `http://127.0.0.1:8765/`。`door` 网页只显示深度，`parkour`、`hybrid` 网页显示 Raw 和 Model。场景入口支持 `--no-data-web` 关闭网页，`--data-web-port` 换端口，`--record` 启动录制。录制落盘 `logs/recordings/`，不保存 RGB 或 MP4，格式见 [监控与录制说明](../monitor/README.md)。

不带 `--profile` 时保留原有共享仿真行为：

```bash
python3 apps/simulation/sim.py --domain 1
./g1 dwaq sim --domain 1       # 或 parkour / hybrid
```

同一用户在同一台机器上一次运行一套仿真。两端默认使用 DDS 域 `1` 和 `lo`。指定 `--domain` 时，两端须使用同一个正数域号。仅运行 DWaQ 且无需相机时，可以在 `dwaq` 的场景入口加 `--no-depth`。

## 启动选项

- `--profile NAME_OR_PATH`：任务档案，决定程序、场景、机器人、策略和相机默认组合。
- `--terrain NAME`：直接选择地形；与档案场景冲突时按提示移除其中一个。
- `--follow`：主视角跟随 torso，初始距离 3 米，仍可用鼠标调整。
- `--depth-preview` / `--no-depth-preview`：显示或关闭相机预览。
- `--no-depth`：禁用相机观测和预览。
- `--fov-debug`：显示深度相机的四条视野边缘射线，需启用预览。
- `--window-backend auto|x11|wayland`：默认 `x11`，`auto` 优先使用现有设置或有 `DISPLAY` 时的 X11。

按手柄的 `Select`（Xbox 的 Back/View 键），或在 MuJoCo 窗口按 `V`，可循环切换跟随视角、机身后方视角和第一人称视角。默认从自由视角开始，首次切换进入跟随。场景进程独立读取视角键，因此只启动场景也可使用。`door` 的第一人称使用档案中的 RGB 相机。切换只影响 MuJoCo 主窗口。

## 地形

| 参数 | 场景 |
| --- | --- |
| `original` | 外部仓库原场景 |
| `platforms` | 10–60 cm 高台 |
| `stones` | 高 10–35 cm、直径 42–52 cm 的交错梅花桩 |
| `gaps` | 宽 20–100 cm 的沟槽 |
| `stairs` | 台阶高 8、12、16 cm 的三组上下楼梯 |
| `hurdles` | 高 10、20、30、40 cm 的窄横栏 |
| `doors` | 正前方 2 m 的右铰链外开门及两侧通墙 |
| `mixed` | 五类障碍的并排独立通道 |

尺寸在 `terrain.json` 中配置。生成场景位于临时目录，退出后清理，不修改外部模型。

door 场景包含 35 kg 被动门板、把手和被动铰链，不新增机器人执行器，`nu` 保持 29。把手摩擦和回位弹簧由生成器覆写，源资产不变。Parkour 的 36×64 射线会过滤静态几何；door 原生 MuJoCo 渲染不受该规则限制。资产来源见 [../../ORIGIN.md](../../ORIGIN.md)。

## 统一相机管线

统一帧可携带：

- Raw：原始米制深度；
- Model：可选的策略预处理深度；
- RGB：可选的独立彩色图像。

Parkour 使用低分辨率 Raw 36×64 和 Model 18×32 双窗口，白框表示模型裁剪区域。door 的 `pipeline: none` 不生成 Model；MuJoCo 本地相机预览可按配置显示 RGB/Depth，Monitor 网页只显示深度。

深度与 RGB 使用独立相机。深度原点对应左红外成像器，RGB 由相机模块中的外参确定。两路使用同一仿真时间和帧序号，不做逐像素对齐或双目噪声仿真。

默认输出为 640×480。垂直视场由相机模块指定，水平视场按实际画幅计算；切换分辨率时自动重算。预览保持宽高比并限制尺寸。

## 机器人头部

`profiles/configs/robots/g1_head_back_20deg.json` 可将整个 `head_link` 绕躯干 +Y 轴后仰，轴心为躯干系 `[0,0,0.320] m`。头部质量和惯量并回躯干，总质量不变，不新增关节。相机安装点和朝向使用同一枢轴变换。

删除头部后仰配置可恢复标准头；`--terrain original` 始终使用外部仓库原样。Parkour 未在该视角下训练，表现需另行验证。

## 操作与验收

键盘为 `9 → p → 等待复位 → r`；`w/s` 前后、`a/d` 横移（DWaQ）、`q/e` 转向、`x` 清零、空格停止。仿真启动时自动识别唯一兼容手柄，也可用 `--joystick PATH`，不支持热插拔重连。

离线推理、空跑和单段仿真不能替代实机运动验收。视频验收只录制 MuJoCo 窗口，不录制整个桌面，默认使用 H.264 `yuv420p`。

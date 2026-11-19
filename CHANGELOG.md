# 改动总结

## 2026-11-19：README 按目录分层

- 根 README 精简为项目概览、构建、通用命令、快速启动和文档导航；删除重复的硬件、遥控器、仿真、相机和部署细节。
- 新增 apps/dwaq、apps/parkour、apps/hybrid、apps/simulation、apps/realsense、apps/monitor、profiles、cameras、robots、common 的专题 README，分别维护对应模块的运行与配置说明。
- 新文档纳入 CMake 安装，保留源码目录和安装目录中的相对链接；仿真启动器仍只从源码运行。

## 2026-11-19：仿真手柄启动时自动识别

- DWaQ、Parkour、Hybrid 和 `g1 joystick` 在不提供 `--joystick` 时自动识别唯一兼容手柄；显式设备路径保持优先，检测到多个兼容手柄时要求明确选择，无兼容手柄时使用键盘。
- 自动识别仅在启动时执行，不支持热插拔重连；设备拔出或读取失败仍进入停止流程，实机仍只使用原配遥控器。
- 验证：全目标构建安装通过；无参数 `g1 joystick` 自动识别当前 Xbox 兼容手柄；door 仿真不传 `--joystick` 时同样自动识别并完成接管、复位、策略启动与停止。

## 2026-11-19：D435 双视角朝向、视场与官方流格式

- 修正场景相机坐标基：RealSense 光学坐标（右、下、前）先转换为 MuJoCo 相机坐标（右、上、后），再写入 XML。此前两路相机朝向后上方，现朝向机器人前方并包含门和地面。
- `cameras/d435_rgbd.json` 分别记录官方 USB 3.1 Gen1 深度 Z16 与 RGB YUY2 的全部标称分辨率和帧率；实机、仿真深度和 RGB 默认均选择 640×480@30，其他格式可按需配置。
- 仿真水平视场按垂直视场与实际画幅计算：默认 4:3 下深度为 73.2613°×58.29°、RGB 为 54.8139°×42.5°，不混用 16:9 标称水平视场。切换分辨率时自动重算。
- 文档补充官方参数来源：内参和外参由设备标定数据经 SDK `get_intrinsics()`、`get_extrinsics_to()` 获取，实际流格式由 `get_stream_profiles()` 枚举；高级模式 JSON 视觉预设不等同于标定文件。
- 验证：28 组深度和 32 组 RGB 官方流格式组合校验通过，非法格式拒绝；默认 Raw/RGB 均为 640×480，前向视场与配置一致。全目标构建、安装和三类离线检查通过；door 完成 DWaQ 接管、复位、0.2 m/s 前进和停止，生成默认格式截图与运动视频供目检。

## 2026-11-18：door 独立 RGB/Depth 双视角

- 相机模块新增可选 RGB 配置：独立场景相机、640×480@30、D435 RGB 视场及深度光学坐标系外参。15 mm 水平间距来自 Intel D400 数据手册图 10-9；旋转为近似值，明确未做目标设备标定，后续可由 RealSense SDK `get_extrinsics_to()` 替换。
- 场景生成按 torso 深度安装变换与深度到 RGB 外参注入两个相机，统一帧增加可选 RGB；同一发布器在主线程同步渲染两路图像，共用帧序号和仿真时间，未配置 RGB 时保持纯深度行为。
- door 显示独立 RGB/Depth 双窗口，保持宽高比并限制总尺寸；Parkour Raw/Model 双窗口不变。本阶段不实现逐像素对齐或双目噪声模型。
- 验证：编译模型测得 RGB 相对深度偏移为 [0.015, 0, 0] m，旋转与配置一致；实际生成 640×480 float32 Raw 和三通道 uint8 RGB，序号和时间一致。door 完成 DWaQ 接管、复位、0.2 m/s 前进与停止，保存双视角截图和运动视频。

## 2026-11-18：仿真任务与统一深度管线解耦

- 新增 dwaq、parkour、hybrid 任务档案；door 显式引用当前 DWaQ 配置。文档明确 door 现阶段只使用 DWaQ 仿真移动，未来将替换为分离上下肢或统一全身策略，本阶段不实现门任务状态机或开门决策。
- 外部仿真器文档路径统一为 `unitree_mujoco_sim`，调试脚本支持 `--mujoco-root`；项目不自动重命名外部目录，本机功能验证仍使用现有等价路径 `unitree_mujoco_parkour`。
- 仿真观测拆为独立兼容层：door 不再继承项目 Parkour 发布器或读取 Parkour 策略配置；原生相机名由相机模块提供。原生渲染保留在查看器主线程，避免物理线程 GLX 上下文错误。
- 深度帧统一携带 Raw、可选 Model、序号、时间和频率。Parkour 保持 Raw 36×64 / Model 18×32 双窗口；door 仅显示 Raw 640×480@30 单窗口，无 Model 和裁剪框。Raw 单窗口按原比例等比缩放，最大 480×360，且不超过视口宽 1/3、高 1/2。
- Hybrid 档案守卫改为 `program=hybrid`；安装目录修正 profiles 嵌套及档案名与启动脚本同名时的解析冲突，启动入口显式提供本包 profiles 路径。
- 验证：全目标构建和安装通过；源码及安装后 dwaq/parkour/hybrid 离线检查通过；门场景离线检查通过；原生渲染实测 640×480、30 Hz、无 Model。DWaQ 原场景、Parkour、Hybrid 仿真接管/复位/策略运行通过；door 完成 DWaQ 接管、复位、0.2 m/s 前进与清零，保存跟随视角视频。目检确认 door Raw 单窗口等比缩小，Parkour/Hybrid 双窗口保持原布局。

## 2026-11-18：任务档案模块化与 door 任务原生深度相机

- 模块化原则落地：策略程序 × 场景资产 × 机器人配置 × 相机参数四轴可独立选择组合；**模块文件按自身命名，不绑定任务名**，任务组合只发生在 `profiles/`。
- 新增目录与文件：`profiles/door.json`（door 任务 = dwaq + doors 场景 + 头部后仰机器人 + 原生 720p 相机；door 现用 dwaq 以避开实机单深度流互斥）；`cameras/d435_depth_1280x720_30hz.json` 与 `cameras/d435_depth_480x270_60hz.json`（相机模块，`pipeline` 字段单一事实来源）；`robots/g1_stock.json` 与 `robots/g1_head_back_20deg.json`（机器人双轨：标准头 / 可调头部转轴后仰 20°，分别用于不同任务）。
- 档案解析与守卫收口 `common/include/g1_control_profile.h`：program 守卫、`policy_config` 覆盖、相机/机器人模块引用解析、实机单深度流互斥守卫（`pipeline: "none"` + 依赖深度的程序 + hardware 启动即报错）。共享解析器（`g1_control_options.h`）新增 `--profile`。
- 相机库参数化（`g1_depth_camera`）：480×270@60 三个编译期常量改为 `Config` 构造参数，`DepthFrame` 携带分辨率元数据，`DepthMeters` 按参数校验分配；`g1_camera_check` 新增 `--camera-config`（720p 原生流的内部消费者，实机验证入口）。`realsense_parkour.json` 回归单一职责（parkour 策略预处理+几何），流参数由 cameras/ 模块承担。
- 仿真接入：`sim.py --profile` 决定场景/机器人/预览默认；`terrain.py::generate_scene` 新增 `head_back` 可选覆盖参数（缺省回落 terrain.json）。
- 验证：全目标编译通过（含 stub）；守卫矩阵通过（dwaq+door 放行、parkour+door 拒绝、档案缺失报错）；场景一致性逐字节通过（robot 模块语义 vs terrain.json 语义）；camera_check 配置接线贯通（实机 720p 帧统计待上架验证）；`sim.py --profile door` 启动行含 profile 且场景/头部角度正确；旧路径与已删符号 grep 无残留。

## 2026-11-18：按键说明收口为词典并表格化

- 按键语义收口为词典（单一事实来源）：新增 `common/include/g1_key_dictionary.h`（动作→键位/说明的 `Lookup` 词典与 `KeyTable/AvailableKeys` 渲染）与 `common/include/g1_text_table.h`（Unicode 两列表格渲染器）。调用点只写动作名，键位写法与说明仅维护一处；键位统一"键盘 / 手柄"顺序（R / A、X / Y 等）。
- DWaQ 与 Parkour/Hybrid 启动即打印全量键位表格（替代原散文式 W/S、Gamepad keys/sticks 说明行；`Ready` 横幅只留状态，时序说明保留并标注复位时长），实机 Unitree 遥控器模式打印遥控器清单行；带 `--joystick` 时设备描述行额外显示枚举详情，手柄摇杆死区（12%）单行说明。
- 状态机各转换点（接管后阻尼等待、复位完成保持、策略运行）打印"当前状态可用按键"表格，按 dry-run/real 区分（如 dry-run 持保持态额外列出 9 / Start 接管）。
- 验证：端到端跑通 9→P→R 全流程（pty 注入按键 + 仿真），启动表格与三种状态表格均正确渲染；parkour pty 下核对通过。注意：重定向到文件时 CycloneDDS 直写会覆盖异步日志头部（既有竞态，终端不受影响）。
- 验证：DWaQ 与 Parkour/Hybrid 启动横幅均完整显示（Parkour 经 pty 终端核对）。注意：重定向到文件时 CycloneDDS 的直写日志会用文件位置指针覆盖异步日志头部（既有竞态，终端输出不受影响）。

## 2026-11-17：门场景把手铰链覆写（消除重力下坠）

- 现象：把手静置下坠实测 5.09°——柄帽重力力矩 0.134 N·m 与源资产回位弹簧（1.5 N·m/rad）的平衡点；把手质量（0.35 kg）为源头 doorman 参数，并非过高。
- 修复（`terrain.json` doors 节新增两个可选键，生成器运行时覆写、资产文件与 SHA256 不变）：
  - `handle_friction`: 0.2 N·m 铰链摩擦损耗；MuJoCo 摩擦约束是软约束，恒定负载下会蠕变回弹簧平衡点，必须同时硬化 `solimpfriction`（阻抗 0.9999）才有效——单加摩擦实测仍下坠 5.05°，硬化后 <0.05°。
  - `handle_stiffness`: 3.0 N·m/rad（源值 1.5 的 2 倍，按用户要求）。
- 验证：最终配置静置 8 s 下坠 +0.047°；删除两个键恢复源资产行为（复测 +5.09°，frictionloss=0、刚度=1.5）；头部后仰与门场景离线检查回归通过。

## 2026-11-17：仿真新增可选头部后仰配置（头+深度相机联动）

- `terrain.json` 新增 `"head": {"back_deg": 20.0}`：生成场景（`--terrain` 非 original）中机器人头部整体后仰，配方复刻 OpenDoor-GraspPlan 分支 `codex/g1-head-pitch20` 的静态安装变换（整块 head_link 绕躯干 +Y 轴后旋，轴心躯干系 [0,0,320] mm；头部 1.036 kg 质量与惯量并入躯干重算，总质量不变）。无新关节，`nu` 保持 29，控制器与关节布局零影响；删除 head 节即恢复标准姿态。
- 深度相机随头部用同一枢点变换（安装点平移 + 光轴下俯 48°→28°），在 `sim.py` 的观测发布器接入 g1_deploy 侧，不改外部仓库文件。parkour 策略未在该视角下训练，表现需实测。
- 离线定量检查通过：旋转探针与 OpenDoor 测试一致（轴上点不动、躯干系 [0,0,0.42] 后移至 [−0.034, 0, 0.414]）；躯干 COM 位移 −3.23 mm/−0.22 mm；惯量特征值全正且与编译模型一致；深度光轴俯角精确 −20°、相机安装点精确绕枢点变换；head 节移除后输出与已验证版本逐值一致；doors/mixed 场景回归通过。站立姿态射线归因：抬头后深度视野无任何机器人自身命中（修复前相机被错误绕躯干原点旋转、甩进肩部区域产生 0.23 m 自命中假象）。
- `sim.py` 新增 `--fov-debug`（需配合 `--depth-preview`）：viewer 场景内从深度相机出发画 4 条半透明绿色射线（实际 36×64 采样扇的四条角边，角上射线角度与视野半角核对一致），标出真实视野边界并随机器人实时移动。

## 2026-11-17：仿真新增门场景 `--terrain doors`

- 从 OpenDoor-GraspPlan 逐字复制门资产（`shared_doorman_v1` MJCF 与两份门配置 JSON，仅基本几何体、无执行器）到 `apps/simulation/assets/doors/`，来源与 SHA256 记入 ORIGIN.md。
- `terrain.py` 新增 doors 场景生成：原始平地上在机器人正前方 2 m 摆右铰链外开门（35 kg 被动门板+把手），两侧通墙（0.16 m 厚、2.4 m 高、外沿 ±4 m，机器人侧与门板面齐平）；资产自带 1 m 小侧墙由正式墙体取代。
- 门为普通被动铰链，`nu` 保持 29，控制器与深度链路接口不变；深度射线过滤静态几何，深度图中仅门板与把手可见。
- `sim.py` 的 `--terrain` choices 与 README 地形表同步新增 doors；离线检查确认机器人 29 关节/87 传感器顺序不变、门关节紧跟其后，原有地形路径回归通过。

## 2026-10-22：程序说明与测试清理

- README 补齐各运行程序的功能、依赖、源码/安装包入口、常用参数与操作方法。
- 移除全部 Python/C++ 自动化测试、算法参考辅助文件、CTest 构建目标及当前使用文档中的测试命令。
- 保留生产程序的离线模型检查、硬件只读检查和相机合成预览入口。下文测试结果属于历史记录，相关测试文件已移除。

## 2026-10-19：RealSense 网页测试与独立 Parkour 推理

- 新增 `camera/realsense_web.py` 和网页：PC2 采集 RealSense RGB/米制深度，外部设备通过 MJPEG 查看画面和相机状态；独立运行不发送机器人命令。
- 新增 `g1_parkour` 双 ONNX 程序和源仓库配套模型/配置。仿真读取外部 `unitree_mujoco` 相机同步观测，实机读取 RealSense 深度、本体 DDS 和 torso secondary IMU。
- 新程序保留默认空跑、Start/9 显式接管、接管清空预览、硬件限制和停止阻尼；原配遥控器沿用 Start/X/A/Select。硬件限制沿用最新远程接口，不恢复估算 PD 力矩或实测关节越界停止。
- 新增相机协议、网页、模型历史/推理、torso 坐标及外部 MuJoCo 联调测试；使用方法和验证边界见 `docs/PARKOUR.md`。未使用 Isaac Sim，未进行机器人或 USB 相机现场验收。

## 2026-10-19：移除估算PD力矩与实测关节越界拦截

按操作者要求，不再计算`Kp*(q_target-q)-Kd*dq+tau`或据此触发停止，移除`joint_effort`配置。
同时取消实测关节位置/速度越界时自动退出；配置范围仍用于限制下发目标。
保留状态完整性、电机故障/温度、IMU倾角、状态超时、目标角及变化率、
真实推理耗时和停止路径；有限指令字段检查独立保留。
如果初始实测角已经越界，首次目标修正优先满足角度边界，可能超过变化率限制。
下方首次整合记录中的估算PD力矩检查已由此项变更取代。

## 2026-10-19：机载空跑、原配遥控器与网页监控整合

本次改动基于独立仓库 `g1_deploy`，将机器人上的调试版本同步回本地，
并将原先位于 `unitree_mujoco_web` 的只读网页监控收入同一仓库。
模型和部署配置的内容保持原样，未将SSH密码、依赖安装目录或机载编译产物纳入版本管理。

### 1. 默认空跑与真实输出分离

- `./g1 run <网卡>`默认关闭输出，不按`9`即可按`p`预览复位、按`r`执行策略推理。
- 空跑读取真实LowState，不模拟机器人跟随目标；姿态或硬件就绪异常显示为警告。
  CRC、机型、有限数据和状态新鲜度仍须通过检查。
- 空跑不创建`rt/lowcmd`发布器、不释放官方运动服务，退出时不发电机阻尼命令。
  目标仅发送到`rt/g1/dry_run_target`，网页显示`OFF · 空跑`。
- 按`9`或所选手柄的Start后先执行完整硬件检查，再申请接管。
  接管会清空预览和策略历史，先进入阻尼等待，必须重新复位后才能运行真实策略。
- 真实输出继续执行关节、温度、倾角、状态超时、目标变化率及估算PD力矩检查。

主要文件：`src/main.cpp`、`include/g1_hardware.h`、`scripts/g1.sh`。

### 2. 宇树原配遥控器接入

新增`./g1 run <网卡> remote`入口，解析LowState中的40字节遥控数据，
与F710输入互斥。最终采用以下映射：

| 输入 | 行为 |
| --- | --- |
| Start | 接管并启用真实输出，与键盘9相同 |
| X | 复位；未接管时仅预览 |
| A | 复位完成后进入策略，进入时摇杆须回中 |
| Select | 软件停止并退出；已接管时先发送约0.5秒阻尼 |
| 左摇杆 | 前后、横移 |
| 右摇杆左右 | 转向 |

进入策略后直接用摇杆控制速度，不需要按住R1；摇杆回中则速度归零，死区12%。
归一化输入为`[ly,-lx,-rx]`，再由YAML限制实际速度范围。
启动时按住的Start/X/A不会触发动作，Select始终优先停止；B、Y没有映射。
仅在显式选择原配遥控器模式时，将原有遥控器A停止位替换为Select停止位。
F710的RB门控及原有按键映射保留。

主要文件：`include/g1_unitree_remote.h`、`include/g1_gamepad.h`、
`src/main.cpp`（对应测试文件已在后续清理中移除）。

### 3. 网页监控并入同一仓库

- `web/joint_monitor.py`：只订阅DDS的Python后端。
- `web/static/joint_monitor.html`：关节表格、IMU、实际/目标曲线及空跑状态。
- `scripts/monitor.sh`：按脚本位置加载资源，支持源码目录及CMake安装目录。
- `web/requirements.txt`、`docs/WEB.md`：可选Python依赖和部署说明。
- `/api/joints`提供当前状态，`/api/stream`使用SSE推送，默认最高50Hz。
- CMake安装包新增`monitor`、`web/`及网页说明，支持`./g1 monitor <网卡> [地址] [端口]`。
  Python依赖由目标机器提供，C++控制程序仍无需Python运行时。

网页来源和保留的机载修改见`ORIGIN.md`。迁移只调整静态文件路径和启动方式，
不包含原仿真工程或无关机器人模型。

### 4. 仓库整理

- 控制源码、模型、配置等19个项目文件已从机器人同步并逐文件校验。
- 新增`.gitattributes`，Shell、Python和HTML使用LF，避免Windows换行破坏Linux启动脚本。
- 扩展`.gitignore`，排除构建、安装、日志、PID、Python依赖、诊断及备份目录。
- README补齐部署、网页、空跑、真实控制和原配遥控器操作步骤。

### 5. 验证记录

验证环境：Ubuntu 22.04 / aarch64 / Python 3.10，真机DDS domain 0，
本次设备网卡`enP8p1s0`。

| 项目 | 结果 |
| --- | --- |
| 机载Release编译、安装及运行库检查 | 通过 |
| C++测试：硬件、F710、原配遥控器、ONNX策略 | 4项通过 |
| 网页测试：HTML/API/SSE、目标优先级、过期状态 | 3项通过 |
| 源码和安装目录的网页启动、29关节实时流 | 通过 |
| 默认空跑复位、推理、速度键和退出 | 通过；DDS发现确认未创建电机指令或MotionSwitcher RPC发布器 |
| 原配遥控器实测摇杆 | 4轴覆盖−1至1，松手回零；约45秒采集4.68万帧，无非法轴值 |
| 原配遥控器模式真机空跑 | 读取真实输入并完成复位预览、策略推理及退出 |

上述自动化和空跑验证没有执行Start接管或真实行走。
操作者随后确认现场调试通过；本总结不补充未记录的行走工况、持续时间或急停验收结果。
Select是软件阻尼停止，不是硬件断电；无线链路是否失联不能仅凭持续更新的LowState判断。

### 使用入口

```bash
cd /home/unitree/g1_deploy/install
./g1 run enP8p1s0 remote
# 空跑：X → 等待目标插值完成 → A，不按Start/9
# 真实控制：Start → X → 等待Holding default pose → A
# 退出：Select；也可使用键盘空格或Ctrl+C
```

另开终端运行`./g1 monitor enP8p1s0 192.168.123.164 8765`，
浏览器访问`http://192.168.123.164:8765/`。完整步骤见`README.md`和`docs/WEB.md`。

# 任务档案

任务档案只负责组合模块，不包含策略实现或场景生成逻辑。模块文件按自身功能命名，任务名只出现在 `profiles/*.json` 和必要文档中。

## 模块分工

```text
任务 = 程序/策略 + 场景 + 机器人 + 相机
```

- 程序/策略：各 `apps/<程序>/config/` 中的模型和部署参数。
- 场景：`scene.terrain` 选择外部原场景或本项目生成的地形。
- 机器人：`robots/*.json` 描述头部安装等机器人属性。
- 相机：`cameras/*.json` 描述流格式、视场、安装外参和处理管线。

## 当前档案

- `dwaq.json`：DWaQ + 原场景 + 标准机器人。
- `parkour.json`：Parkour + 原场景 + 标准机器人 + Parkour 相机管线。
- `hybrid.json`：混合策略 + 标准机器人 + Parkour 相机管线。
- `door.json`：门场景 + 后仰机器人 + D435 RGB/Depth + 当前 DWaQ。

## 字段

```json
{
  "program": "dwaq",
  "policy_config": "../apps/dwaq/config/g1_locomotion.json",
  "scene": {"terrain": "doors"},
  "robot": {"config": "../robots/g1_head_back_20deg.json"},
  "camera": {"config": "../cameras/d435_rgbd.json"}
}
```

- `program`：程序守卫，必须与实际控制器一致；Hybrid 使用 `hybrid`。
- `policy_config`：可选策略配置覆盖；缺省使用程序标准配置。
- `scene.terrain`：场景选择，档案优先，冲突的直接 `--terrain` 会报错。
- `robot.config`：机器人模块；`head_back_deg` 覆盖 `terrain.json` 的头部默认值。
- `camera.config`：相机模块；`pipeline` 以模块文件为单一事实来源。

相对路径以档案文件所在目录解析。源码目录和安装目录均支持 `--profile <名称或路径>`；不带 `--profile` 时使用原有默认配置。

实机 Parkour/Hybrid 需要处理后的策略深度，`pipeline: none` 的原生深度流与其在单 D435 上互斥，档案守卫会拒绝不兼容组合。door 当前使用 DWaQ，不消费策略深度，因此没有该冲突。

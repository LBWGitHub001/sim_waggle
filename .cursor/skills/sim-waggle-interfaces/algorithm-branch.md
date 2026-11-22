# 算法分支工作流

注意主分支是一个总体的基，其他的算法尝试都是基于 main 的一系列分支。

## 分支策略

| 分支 | 职责 |
|---|---|
| `main` | 仿真平台 + `tracker::Target` EKF + Aimer。接口变更只能是加性、向后兼容 |
| `exp/<name>` | 一个算法一次尝试。从 **当前 `main`** 拉出，不要从另一个 exp 叠 |
| 现有例子 | `exp/ego-acc`：`Target::control(u)`，TrackerNode 里 `predict → control(ego_acc) → update` |

不要：在 `main` 上直接改 EKF 模型；从脏工作区切分支；把失败实验 squash 进 main；再写一层 `*Tracker` 包装类。

**每一步都 `git commit`，然后立刻 `git push`。** 开分支、改 Target、接线、扩消息——每做完一件立刻提交并推送，不要攒。多分支并行时每个分支各自 commit + push。

**未经用户点名不改 OmniScope。**

## 新建实验

```
git checkout main
git pull
git checkout -b exp/<short-name>
```

命名用算法意图，例如 `exp/ego-acc`、`exp/s2-ego-velocity`。

## 改哪些文件

**通常改（分支上）：**

- `tracker/target.hpp/.cpp` — 过程模型、`control(u)`、观测
- `tracker/tracker_node.cpp` — 调用序（如 predict / control / update）
- `app/main.cpp` — 仅当 YAML 要多读字段
- `configs/sim_waggle.yaml` — 可选 `algo:` 节
- 若需自车状态：`common/types.hpp` 给 `StampedObservation` **追加字段**；`sim/sim_node.cpp::publish` 赋值

**通常不要改：**

- `sim/kinematics.*` 物理
- `aimer/` 火控（除非实验就是火控）
- 工作系 = gimbal
- 不要引入 `ITracker` / `virtual` / `BaselineTracker` / `EgoControlTracker`
- `thirdparty/OmniScope`（除非用户点名）
- 已有 plot 键名、Marker `ns/id`（可新增）

**main 上才动的事：** 通道语义、节点生命周期、包隔离、把已验证算法升为默认实现。

## 自车状态怎么送进滤波

`update(obs)` 只有 `ArmorObservation`。需要 `p_g/v_g/ω_c` / `ego_acc` 时：

1. 在 `StampedObservation` 末尾加字段（带默认值）
2. `SimNode::publish` 从 `gt.gimbal` / `gt.chassis` 填写
3. 在本分支的 `TrackerNode::run` 里把新字段交给 `Target::control` 等，不要为此引入包装类
4. 新 `plot` 必须用 `msg.stamp_ns` 的 `t_mark` 重载，不要 `omniscope::now_ns()`

构造期常量可以直接从 YAML 注入，不必走消息。时变量必须走消息。

## `exp/ego-acc` 模式

不要恢复 `EgoControlTracker`。在 `Target` 上加 `control(u)`，`TrackerNode::run`：

```
predict(dt)
control(msg.ego_acc)   // 或等价控制输入
update(msg.obs)
```

YAML 的 `m_ax` / `noise_acc_std` 等只在该分支读取。

## 验收

- 能编过：`cmake --build cmake-build-debug`
- 默认 yaml 跑几秒不发散；终端能看到 `cmd_yaw` 且 `valid=true`（小陀螺稳态）
- OmniScope 上 `obs_*` / `true_*` / `kal_*` / `cmd_*` 仍在；新键可另开
- 3D：Kalman 黄心蓝甲（`frame_id=gimbal`）；真值全绿在 `robot`；`aim.lead` 品红；不要在 main 里 plot
- 相对 main 的 diff 应集中在 Target / TrackerNode；若动了 `common/types.hpp`，字段必须有默认值

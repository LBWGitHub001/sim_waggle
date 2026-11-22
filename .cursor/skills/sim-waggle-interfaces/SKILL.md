---
name: sim-waggle-interfaces
description: >-
  Documents sim_waggle package interfaces, message types,
  coordinate frames, OmniScope telemetry keys, the main-as-base / algorithm-branch
  workflow, and the rule that every step is git-committed and pushed. Use when adding a
  tracker algorithm, branching from main, changing SimNode/TrackerNode/AimerNode,
  YAML config, plot keys, 3D markers, metrics, common types, or wiring app/main.cpp.
---

# sim_waggle 接口与分支约定

注意主分支是一个总体的基，其他的算法尝试都是基于 main 的一系列分支。

`main` 是稳定仿真平台（物理、观测、通道、基线 EKF、火控）。任何新算法都从 `main` 开分支，在分支上改 `tracker::Target` / `TrackerNode::run`（例如加 `control(u)`）；不要虚基类、不要继承多态、不要再包一层 tracker 类；不要把未验证算法合回 `main`。

## OmniScope：未经点名不得改

**没有用户明确点名 OmniScope / Host / SDK / 协议时，禁止改 `thirdparty/OmniScope` 的任何文件。** 不要「顺便」改 Host UI、协议、C++ SDK。本仓库只用已有的 `omniscope::plot` / `plot_markers` / `plot_tfs`。

## Git：每一步都提交并推送

在本仓库做任何改动时，**每完成一个独立步骤立刻 `git commit`，然后立刻 `git push`**。不要攒到最后一次性交，不要等用户再说「提交」或「推送」。

- 一步 = 一组语义完整、尽量可编译的改动
- 同时改 `main` 和其他算法分支时，**每个分支各自 commit + push**
- 提交信息写 why，1–2 句；不要空提交；不要 `--amend` 已经推过的提交
- 只有用户明确要求改 OmniScope 时：先在子模块里 commit 并 `git push`，再在本仓库更新 submodule 指针并 commit + push

## 先读哪份

| 任务 | 读 |
|---|---|
| 换滤波 / 新补偿算法 | 本文件「算法接入」+ [algorithm-branch.md](algorithm-branch.md) |
| 改消息、节点、YAML、CMake | [interfaces.md](interfaces.md) |
| 改曲线名、Marker、坐标系、误差口径 | [frames-and-telemetry.md](frames-and-telemetry.md) |

## 包边界（不可破）

```
sim/  ──Channel<StampedObservation>──►  tracker/  ──Channel<TrackerOutput>──►  aimer/
  │                                        │                                      │
  └──LatestSlot<GroundTruth>───────────────┴──────────────────────────────────────┘
                                                                              │
                                                              LatestSlot<GimbalCommand>
                                                                              │
app/main.cpp  只接线三节点 + YAML + 起停。禁止 plot / plot_image / plot_markers / plot_tfs。
```

- `sim/`、`tracker/`、`aimer/` 是独立 CMake 包，**编译期互相看不见头文件**。唯一耦合是 `common/`（头文件：`types.hpp` / `channel.hpp` / `frames.hpp`）。
- `common/` 无滤波、无火控。`tinympcstatic` 只链 `aimer`。
- 禁止 `#include` 对面包的头。需要新数据就**加性**扩 `common` 消息。
- 遥测：`omniscope::plot` / `plot_markers` / `plot_tfs` 只写在 **SimNode / TrackerNode / AimerNode**。main 禁止 `plot` 和 `plot_image`。

## 坐标系（必守）

四系：

| 名字 | 角色 |
|---|---|
| **odom** | 场地惯性系。运动学积分、TF 中间件。日常滤波 / plot / 误差 **禁止**直接用 odom 坐标 |
| **gimbal** | 原点=云台中心，**轴向与 odom 平行、不转**。观测、EKF、`obs_*`/`true_*`/`kal_*`/`err_*`、Kalman Marker |
| **camera** | 与 gimbal 同原点，轴向随 look-at yaw。只给光轴 TF |
| **robot** | 敌车体。装甲板绑在此系；robot 在 odom 中转动 |

`GroundTruth` 的 chassis / gimbal.pos / camera / target 是 odom。日常使用 `common::express_target_in_gimbal(gt)`（[`frames.hpp`](../../../common/frames.hpp)）。

EKF 吃 `xyz_in_gimbal`（`armor_odom - gimbal.pos`，不乘 look-at）。`xyz_in_world` 是装甲 odom 坐标，滤波和 plot 不用。观测 `armor_id` = 仿真 `visible_armor_id`，不做角度匹配。

误差在 **TrackerNode**：Kalman − gimbal 系敌车真值。杆臂公转会出现在 `true_center_y`；要看场地里敌车几乎静止，把 Host 3D 查看系切到 `odom`。切 `camera` 看光轴锁定。

细节见 [frames-and-telemetry.md](frames-and-telemetry.md)。

## 算法接入（分支上做）

1. `git checkout main && git checkout -b exp/<algo-name>`
2. 改 `tracker::Target`（过程模型 / `control(u)` / 观测）和/或 `TrackerNode::run` 的调用序
3. 需要自车运动时：**加性**扩展 `StampedObservation`（如 `gimbal_pos/vel`、`chassis_w`、`ego_acc`），`SimNode::publish` 填；在该分支 `TrackerNode::run` 里交给 `Target`
4. `TargetEstimate` 必须填 `center/velocity/yaw/w/radius/armor_xyza_list`；`x.size() >= 11`；`jumped = true`（Aimer coming/leaving 依赖）
5. 允许新增 `omniscope::plot` 键；**禁止改名**已有键（`obs_*` / `true_*` / `kal_*` / `cmd_*`）
6. **不要**再写 `BaselineTracker` / `EgoControlTracker` 这类包装类

`Target` 约定方法：

```
init(obs)
predict(dt)
update(obs)            // 未 init 时内部应自动 init
estimate() -> TargetEstimate
```

实验分支可加 `control(u)`。`TrackerNode` 调用序：首帧只 `init`；之后 `predict(sim_t - last_t)` + `update`（ego-acc 分支在中间插 `control`）；每帧 `estimate()`，用 `gt_in` 做误差，`Channel<TrackerOutput>` 推给 AimerNode。不丢帧。

完整检查单见 [algorithm-branch.md](algorithm-branch.md)。

## 节点与通道

| 对象 | 角色 |
|---|---|
| `Channel<T>` | FIFO，`push` 不阻塞；`close` 后排空才返回 false |
| `LatestSlot<T>` | keep_last(1) |
| `SimNode` | fps 推进；先发 t=0；话题：`obs_out` / `gt_out`；观测 RNG 种子 42 |
| `TrackerNode` | 直接持有 `tracker::Target`；消费 FIFO；读 `gt_in` 做误差；写 `est_out`；发 `kal_*`/`err_*`/Kalman Marker |
| `AimerNode` | 直接持有 `Planner`；`plan(est)`；写 `cmd_out`；发 `cmd_*` 与 `aim.lead` |
| 收尾 | `sim.stop(); sim.join(); tracker.join(); aimer.join();`（sim 关 obs 通道，tracker 关 est 通道） |

仿真时钟 `sim_t = frame * dt`，与墙钟无关。

## YAML（`configs/sim_waggle.yaml`）

`app/main.cpp::load_config()` 一处解析。节：`simulation` / `my_robot` / `target_robot` / `measurement` / `aimer` / `planner` / `ekf` / `remote_logger`。字段表见 [interfaces.md](interfaces.md)。

## 基线状态（`tracker::Target`，11 维）

`x = [xc, vx, yc, vy, zc, vz, yaw, w, r, l, h]`

- CV + 常角速度；观测 4 维 YPDA `[yaw, pitch, dist, armor_yaw]`
- 观测板号 = `obs.armor_id`（仿真可见板），常对角 \(R=\mathrm{diag}(r_{yaw},r_{pitch},r_d,r_{yaw})\)
- `l=r2-r1`，`h=z2-z1`（四装甲奇偶板：对侧同 r/z，邻侧 `r+l` / `zc+h`）
- 过程噪声 `EkfParams`；约瑟夫形式协方差（`tools::ExtendedKalmanFilter`）
- 仿真 `armor_in_robot(i,n,r,l,h)` 必须与 `h_armor_xyz` 同一套几何

火控是平台能力（main）：`common::Aimer` 出无约束射击角，`common::Planner`（TinyMPC）出可行 `cmd.yaw/pitch`。不做开火判定，不驱动仿真 camera。

## 硬约束

- 每完成一步就 `git commit` 并立刻 `git push`
- 工作系 = 方向不变的 gimbal；camera 只做 TF；odom 只作运动学/TF 中间件
- `omniscope::plot` / `plot_markers` / `plot_tfs` 只写在三个节点；误差分析在 TrackerNode；`cmd_*` / `aim.lead` 在 AimerNode
- 不让 `sim/` / `tracker/` / `aimer/` 互相看见头文件
- **未经用户点名不改 OmniScope**
- 不把实验合进 `main`，除非明确要求提升为新基线
- 不引入虚函数 / 继承多态 / 包装 tracker 类切换算法
- `aim.lead` 用 `GimbalCommand.aim_xyz`（弹道提前点）

---
name: sim-waggle-interfaces
description: >-
  Documents sim_waggle package interfaces, concrete tracker types, message types,
  coordinate frames, OmniScope telemetry keys, the main-as-base / algorithm-branch
  workflow, and the rule that every step is git-committed and pushed. Use when adding a tracker
  algorithm, branching from main, changing
  SimNode/TrackerNode, YAML config, plot keys, 3D markers, metrics, common types,
  or wiring app/main.cpp.
---

# sim_waggle 接口与分支约定

注意主分支是一个总体的基，其他的算法尝试都是基于main的一系列分支。

`main` 是稳定仿真平台（物理、观测、通道、可视化、基线 EKF）。任何新算法都从 `main` 开分支，在分支上换成该分支自己的具体 tracker 类；不要虚基类、不要继承多态；不要把未验证算法合回 `main`。

## Git：每一步都提交并推送

在本仓库做任何改动时，**每完成一个独立步骤立刻 `git commit`，然后立刻 `git push`**。不要攒到最后一次性交，不要等用户再说「提交」或「推送」。

- 一步 = 一组语义完整、尽量可编译的改动（例如删掉 `ITracker`、改世界系、注入控制量）
- 同时改 `main` 和其他算法分支时，**每个分支各自 commit + push**
- 提交信息写 why，1–2 句；不要空提交；不要 `--amend` 已经推过的提交
- 改了 `thirdparty/OmniScope`：先在子模块里 commit 并 `git push`，再在本仓库更新 submodule 指针并 commit + push

## 先读哪份

| 任务 | 读 |
|---|---|
| 换滤波 / 新补偿算法 | 本文件「算法接入」+ [algorithm-branch.md](algorithm-branch.md) |
| 改消息、节点、YAML、CMake | [interfaces.md](interfaces.md) |
| 改曲线名、Marker、坐标系、误差口径 | [frames-and-telemetry.md](frames-and-telemetry.md) |
| 问题成因与候选方案 S1–S5 | `docs/analysis_and_solutions.md` |

## 包边界（不可破）

```
sim/  ──Channel<StampedObservation>──►  tracker/
  │                                        │
  └──LatestSlot<GroundTruth>──────────────►│  （TrackerNode 做误差；main 只读 HUD）
  └──LatestSlot<GroundTruth / obs>──┐      └──LatestSlot<TrackerOutput>──┐
                                    ▼                                    ▼
                                 app/main.cpp  （接线 + plot_image）
```

- `sim/` 与 `tracker/` 是独立 CMake 包，**编译期互相看不见头文件**。唯一耦合是 `common/`。
- 禁止 `#include` 对面包的头。需要新数据就**加性**扩 `common` 消息，不要开第二套类型。
- `thirdparty/OmniScope`：未点名时不改 SDK 源码；Host UI / 协议可按任务**加性**扩展。遥测：`omniscope::plot` / `plot_markers` / `plot_tfs` 只写在 **SimNode / TrackerNode**；main 只允许 `plot_image`。

## 坐标系（必守）

三系：

| 名字 | 角色 |
|---|---|
| **odom** | 场地惯性系。运动学积分、TF 中间件。日常滤波 / plot / 误差 **禁止**直接用 odom 坐标 |
| **gimbal** | 云台/相机系（原点=云台，轴向随 yaw）。观测、EKF、`obs_*`/`true_*`/`kal_*`/`err_*`、Kalman Marker |
| **robot** | 敌车体。装甲板绑在此系；robot 在 odom 中转动 |

`GroundTruth` 的 chassis/gimbal/target 是 odom。日常使用 `common::express_target_in_gimbal(gt)`（[`frames.hpp`](../../../common/frames.hpp)）。

EKF 吃 `xyz_in_gimbal`。`xyz_in_world` 是装甲 odom 坐标，滤波和 plot 不用。

误差在 **TrackerNode**：Kalman − gimbal 系敌车真值。Kalman 会跟上云台里的表观运动；要看场地里敌车几乎静止，把 Host 3D 查看系切到 `odom`。

`omniscope::plot` 禁止写在 `app/main.cpp`。

细节见 [frames-and-telemetry.md](frames-and-telemetry.md)。

## 算法接入（分支上做）

1. `git checkout main && git checkout -b exp/<algo-name>`
2. 在 `tracker/` 新增具体 tracker 类（无基类、无 `virtual`），注册进 `tracker/CMakeLists.txt`
3. 把 `TrackerNode` 持有的具体类型、以及 `app/main.cpp` 的构造改成新类（构造参数可从 YAML 加字段）
4. 需要自车运动时：**加性**扩展 `StampedObservation`（如 `gimbal_pos/vel`、`chassis_w`、`ego_acc`），`SimNode::publish` 填；在该分支 `TrackerNode::run` 里把新字段交给具体 tracker
5. `TargetEstimate` 必须填 `center/velocity/yaw/w/radius/armor_xyza_list`；`x.size() >= 11`（metrics / plot / marker 都卡这个）
6. 允许新增 `omniscope::plot` 键；**禁止改名**已有键（`obs_*` / `true_*` / `kal_*`）

具体 tracker 约定方法（不是虚接口，只是各分支自己实现同名成员）：

```
init(obs, t)
predict(dt)
update(obs)            // 未 init 时内部应自动 init
estimate() -> TargetEstimate
predict_aim_point(flight_time)
diverged() / last_nis() / name()
```

`TrackerNode` 调用序（不要改语义）：首帧只 `update`；之后 `predict(sim_t - last_t)` + `update`；每帧 `estimate()`，用 `gt_in` 做误差，写入 `LatestSlot<TrackerOutput>`（含 `metrics`）。不丢帧。

接线模板（`app/main.cpp`）：

```cpp
auto tracker = std::make_unique<tracker::YourTracker>(...);
tracker::TrackerNode tracker_node(
  std::move(tracker), obs_channel, est_slot, gt_slot, cfg.sim.bullet_speed);
```

完整检查单见 [algorithm-branch.md](algorithm-branch.md)。

## 节点与通道

| 对象 | 角色 |
|---|---|
| `Channel<T>` | FIFO，`push` 不阻塞；`close` 后排空才返回 false |
| `LatestSlot<T>` | keep_last(1)，可视化只读拷贝 |
| `SimNode` | fps 推进；先发 t=0；话题：`obs_out` / `gt_out` / `obs_latest` |
| `TrackerNode` | 注入 `unique_ptr<本分支具体 Tracker>`；消费 FIFO；读 `gt_in` 做误差；写 `est_out`（含 `metrics`）；发 `kal_*`/`err_*`/Kalman Marker |
| 收尾 | `sim.stop(); sim.join(); tracker.join();`（sim 关通道） |

仿真时钟 `sim_t = frame * dt`，与墙钟无关。传感器 RNG 种子 42，可复现。

## YAML（`configs/sim_waggle.yaml`）

`app/main.cpp::load_config()` 一处解析。节：`simulation` / `my_robot` / `target_robot` / `measurement` / `ekf` / `remote_logger`。字段表见 [interfaces.md](interfaces.md)。

## 基线状态（`auto_aim::Target`，11 维）

`x = [xc, vx, yc, vy, zc, vz, yaw, w, r, l, h]`

- CV + 常角速度；观测 4 维 YPDA `[yaw, pitch, dist, armor_yaw]`
- `l=r2-r1`，`h=z2-z1`（四装甲奇偶板）
- 过程噪声 `EkfParams`；约瑟夫形式协方差（`tools::ExtendedKalmanFilter`）

main 上的实现是 `tracker::BaselineTracker`，忠实还原 auto_aim，**不要在 main 上改它的模型**。补偿算法另写类。

## 硬约束

- 每完成一步就 `git commit` 并立刻 `git push`（见上文「Git：每一步都提交并推送」）
- 工作系 = gimbal；odom 只作运动学/TF 中间件；不要把 odom 坐标喂给 EKF 或 `plot`
- `omniscope::plot` / `plot_markers` / `plot_tfs` 只写在 SimNode / TrackerNode；误差分析在 TrackerNode
- 不让 `sim/` 看见 `tracker/`，反之亦然
- 不改 OmniScope 源码（除非任务明确要求改 Host / 加性协议）
- 不把实验合进 `main`，除非明确要求提升为新基线
- 可视化线程只读 LatestSlot，不碰节点内部状态
- 不引入虚函数 / 继承多态切换算法（每个分支一个具体 tracker 类型）
- `aim.lead` 用 `common::compute_aim_lead_point(est, flight_time)`，不要跨线程调 tracker 的 `predict_aim_point`

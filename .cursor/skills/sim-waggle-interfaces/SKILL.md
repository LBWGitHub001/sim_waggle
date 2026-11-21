---
name: sim-waggle-interfaces
description: >-
  Documents sim_waggle package interfaces, concrete tracker types, message types,
  coordinate frames, OmniScope telemetry keys, the main-as-base / algorithm-branch
  workflow, and the rule that every step is git-committed. Use when adding a tracker
  algorithm, branching from main, changing
  SimNode/TrackerNode, YAML config, plot keys, 3D markers, metrics, common types,
  or wiring app/main.cpp.
---

# sim_waggle 接口与分支约定

注意主分支是一个总体的基，其他的算法尝试都是基于main的一系列分支。

`main` 是稳定仿真平台（物理、观测、通道、可视化、基线 EKF）。任何新算法都从 `main` 开分支，在分支上换成该分支自己的具体 tracker 类；不要虚基类、不要继承多态；不要把未验证算法合回 `main`。

## Git：每一步都提交

在本仓库做任何改动时，**每完成一个独立步骤立刻 `git commit`**。不要攒到最后一次性交，不要等用户再说「提交」。

- 一步 = 一组语义完整、尽量可编译的改动（例如删掉 `ITracker`、改世界系、注入控制量）
- 同时改 `main` 和其他算法分支时，**每个分支各自提交**
- 提交信息写 why，1–2 句；不要空提交；不要 `--amend` 已经推过的提交
- `git push` 仅在用户明确要求时

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
  └──LatestSlot<GroundTruth / obs>──┐      └──LatestSlot<TrackerOutput>──┐
                                    ▼                                    ▼
                                 app/main.cpp  （唯一接线层 + 3D/图像）
```

- `sim/` 与 `tracker/` 是独立 CMake 包，**编译期互相看不见头文件**。唯一耦合是 `common/`。
- 禁止 `#include` 对面包的头。需要新数据就**加性**扩 `common` 消息，不要开第二套类型。
- `thirdparty/OmniScope` 只读 SDK。遥测直接 `omniscope::plot` / `plot_markers` / `plot_image`，不设 bridge。

## 坐标系（必守）

世界系全工程唯一：

```
原点：绑定正在旋转的云台中心（随云台平移）
轴向：与惯性系平行，不随云台旋转
```

`xyz_in_world` / EKF / `kal_*` / 3D Marker / 相机渲染用 `GroundTruth::target` 都在此系。因此 `gimbal.pos ≡ 0`、`gimbal.vel ≡ 0`，底盘绕原点公转。

`xyz_in_gimbal` 才是随云台 yaw 转的相机系。不要再减一遍 `gimbal.pos`。

敌车平移真值用 **`GroundTruth::target_inertial`（辅助惯性系 / 场地）**，不要用世界系表观 `target.center/vel`。把云台公转写进真值会让 Kalman 和真值贴在一起，不像真实自瞄。

误差：**Kalman 状态 − `target_inertial`**。默认 `d=0.05` m、`ω=12` rad/s 时位置差约杆臂圆（cm 级）、速度差约 `ω|d|=0.6` m/s。`err_*` 由 `app/main.cpp` 在 metrics 之后 `plot`。

细节与 plot/marker 表见 [frames-and-telemetry.md](frames-and-telemetry.md)。

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

`TrackerNode` 调用序（不要改语义）：首帧只 `update`；之后 `predict(sim_t - last_t)` + `update`；每帧 `estimate()` 写入 `LatestSlot`。不丢帧。

接线模板（`app/main.cpp`）：

```cpp
auto tracker = std::make_unique<tracker::YourTracker>(...);
tracker::TrackerNode tracker_node(std::move(tracker), obs_channel, est_slot);
```

完整检查单见 [algorithm-branch.md](algorithm-branch.md)。

## 节点与通道

| 对象 | 角色 |
|---|---|
| `Channel<T>` | FIFO，`push` 不阻塞；`close` 后排空才返回 false |
| `LatestSlot<T>` | keep_last(1)，可视化只读拷贝 |
| `SimNode` | fps 推进；先发 t=0；话题：`obs_out` / `gt_out` / `obs_latest` |
| `TrackerNode` | 注入 `unique_ptr<本分支具体 Tracker>`；消费 FIFO，写 `est_out` |
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

- 每完成一步就 `git commit`（见上文「Git：每一步都提交」）
- 不改世界系定义（原点=云台中心，轴向固定）；敌车平移真值用 `target_inertial`，不用表观 `target.vel`
- 不让 `sim/` 看见 `tracker/`，反之亦然
- 不改 OmniScope 源码
- 不把实验合进 `main`，除非明确要求提升为新基线
- 可视化线程只读 LatestSlot，不碰节点内部状态
- 不引入虚函数 / 继承多态切换算法（每个分支一个具体 tracker 类型）
- `aim.lead` 用 `common::compute_aim_lead_point(est, flight_time)`，不要跨线程调 tracker 的 `predict_aim_point`

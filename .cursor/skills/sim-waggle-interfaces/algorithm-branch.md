# 算法分支工作流

注意主分支是一个总体的基，其他的算法尝试都是基于main的一系列分支。

## 分支策略

| 分支 | 职责 |
|---|---|
| `main` | 仿真平台 + `BaselineTracker`。接口变更只能是加性、向后兼容 |
| `exp/<name>` | 一个算法一次尝试。从 **当前 `main`** 拉出，不要从另一个 exp 叠 |
| 现有例子 | `exp/ego-acc`：自车 gimbal xy 加速度作 EKF 控制量 `u`（`EgoControlTracker`） |

不要：在 `main` 上直接改 EKF 模型；从脏工作区切分支；把失败实验 squash 进 main。

**每一步都 `git commit`，然后立刻 `git push`。** 开分支、加 tracker、接线、扩消息、改节点——每做完一件立刻提交并推送，不要攒。多分支并行时每个分支各自 commit + push。改 OmniScope 子模块时：子模块先 commit/push，再提交本仓库的 submodule 指针并 push。

## 新建实验

```
git checkout main
git pull
git checkout -b exp/<short-name>
```

命名用算法意图，例如 `exp/s1-world-frame`、`exp/s2-ego-velocity`、`exp/s3-harmonic-aug`。方案含义见 `docs/analysis_and_solutions.md`：

- S1：把观测变到 odom（工作系是 gimbal；不要把 odom 坐标当日常滤波输入，除非实验明确要切系）
- S2：过程模型吃已知 `v_g`（需把自车速度送进 tracker）
- S3：增广谐波/杆臂状态
- S4：只调 Q/R — 文档已证不能消几何项，勿当主方案
- 瞄准级事后加 `v_g*T` — 文档已证更差，不要再试除非为了复现反例

## 改哪些文件

**通常改（分支上）：**

- `tracker/<algo>_tracker.hpp/.cpp` — 新的具体 tracker 类（无基类）
- `tracker/tracker_node.hpp/.cpp` — 把持有类型改成新类
- `tracker/CMakeLists.txt` — 加入 `.cpp`
- `app/main.cpp` — 构造新 tracker（`#include` + `make_unique`）
- `configs/sim_waggle.yaml` — 可选 `algo:` 节
- 若需自车状态：`common/types.hpp` 给 `StampedObservation` **追加字段**；`sim/sim_node.cpp::publish` 赋值

**通常不要改：**

- `sim/kinematics.*` 物理（odom 积分 / robot 绑装甲）
- 工作系 = gimbal；odom 只作 TF 中间件。S1 若把滤波改到 odom，须在提交说明写清
- 不要引入 `ITracker` / `virtual` / 继承多态来切换算法
- `thirdparty/OmniScope`
- 已有 plot 键名、Marker `ns/id`（可新增）

**main 上才动的事：** 通道语义、节点生命周期、包隔离、把已验证算法升为默认实现。

## 自车状态怎么送进滤波

`update(obs)` 只有 `ArmorObservation`。需要 `p_g/v_g/ω_c` 时：

1. 在 `StampedObservation` 末尾加字段（带默认值，旧代码仍编译）
2. `SimNode::publish` 从 `gt.gimbal` / `gt.chassis` 填写
3. 在本分支的 `TrackerNode::run` 里把新字段交给具体 tracker（例如 `control(msg.ego_acc)`），不要为此引入虚基类

不要为了传 ω 去 include `sim/kinematics.hpp`。

构造期常量（固定 `chassis_w`、`gimbal_offset`）可以直接从 YAML 注入构造函数，不必走消息。时变量必须走消息。

## 新 tracker 最小骨架

```cpp
class CompensatedTracker {
public:
  void init(const common::ArmorObservation& obs, double t);
  void predict(double dt);
  void update(const common::ArmorObservation& obs); // 空则 init
  common::TargetEstimate estimate() const;
  Eigen::Vector3d predict_aim_point(double flight_time) const;
  bool diverged() const;
  double last_nis() const;
  std::string name() const { return "CompensatedTracker"; }
};
```

`estimate()` 至少填：`x`(≥11)、`center`、`velocity`、`yaw`、`w`、`radius`、`armor_xyza_list`、`last_nis`、`diverged`。
`predict_aim_point` 与 `compute_aim_lead_point` 公式保持一致，除非算法故意改瞄准律（须在提交说明写清）。

接线只改一处：

```cpp
auto tracker = std::make_unique<tracker::CompensatedTracker>(...);
tracker::TrackerNode tracker_node(
  std::move(tracker), obs_channel, est_slot, gt_slot, cfg.sim.bullet_speed);
```

## 验收

- 能编过：`cmake --build cmake-build-debug`
- 默认 yaml 跑几秒不发散；终端 `kal-true` 误差与 HUD / `err_*` 曲线同口径
- OmniScope 上 `obs_*` / `true_*` / `kal_*` 仍在；新键可另开
- 3D：Kalman 黄心蓝甲（`frame_id=gimbal`）；真值全绿在 `robot`；camera 光轴在 `camera`；不要在 main 里 plot
- 相对 main 的 diff 应集中在 tracker 新文件 + 少量接线；若动了 `common/types.hpp`，字段必须有默认值

## 和无界面 benchmark

`experiments/bench.cpp` 是独立定量脚本（不经 OmniScope），直接调 `KinematicsEngine` / `CameraSensor` / `BaselineTracker`。分支上可比对 BASE vs 新 variant；不要把它的补偿逻辑复制进 `main` 的节点路径，除非升格算法。

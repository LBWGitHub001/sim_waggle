# 算法分支工作流

注意主分支是一个总体的基，其他的算法尝试都是基于main的一系列分支。

## 分支策略

| 分支 | 职责 |
|---|---|
| `main` | 仿真平台 + `BaselineTracker`。接口变更只能是加性、向后兼容 |
| `exp/<name>` | 一个算法一次尝试。从 **当前 `main`** 拉出，不要从另一个 exp 叠 |
| 现有例子 | `high_freq`（曾作实验，现与 main 同提交）；`exp/kalman-only-markers`（可视化，非滤波） |

不要：在 `main` 上直接改 EKF 模型；从脏工作区切分支；把失败实验 squash 进 main。

## 新建实验

```
git checkout main
git pull
git checkout -b exp/<short-name>
```

命名用算法意图，例如 `exp/s1-world-frame`、`exp/s2-ego-velocity`、`exp/s3-harmonic-aug`。方案含义见 `docs/analysis_and_solutions.md`：

- S1：把观测变到底盘/场地惯性系（当前世界原点已是云台，`gimbal.pos≡0`，不要再加一遍）
- S2：过程模型吃已知 `v_g`（需把自车速度送进 tracker）
- S3：增广谐波/杆臂状态
- S4：只调 Q/R — 文档已证不能消几何项，勿当主方案
- 瞄准级事后加 `v_g*T` — 文档已证更差，不要再试除非为了复现反例

## 改哪些文件

**通常改（分支上）：**

- `tracker/<algo>_tracker.hpp/.cpp` — 新 `ITracker`
- `tracker/CMakeLists.txt` — 加入 `.cpp`
- `app/main.cpp` — 构造新 tracker（`#include` + `make_unique`）
- `configs/sim_waggle.yaml` — 可选 `algo:` 节
- 若需自车状态：`common/types.hpp` 给 `StampedObservation` **追加字段**；`sim/sim_node.cpp::publish` 赋值

**通常不要改：**

- `sim/kinematics.*` 物理
- 世界系定义（原点=云台、轴向固定）；S1 若改到场地惯性系，须在提交说明写清
- `ITracker` 虚函数签名（要新输入就扩消息，或给派生类加构造参数）
- `thirdparty/OmniScope`
- 已有 plot 键名、Marker `ns/id`（可新增）

**main 上才动的事：** 通道语义、节点生命周期、包隔离、把已验证算法升为默认实现。

## 自车状态怎么送进滤波

`ITracker::update` 只有 `ArmorObservation`。需要 `p_g/v_g/ω_c` 时：

1. 在 `StampedObservation` 末尾加字段（带默认值，旧代码仍编译）
2. `SimNode::publish` 从 `gt.gimbal` / `gt.chassis` 填写
3. 新 tracker 在 `TrackerNode` 里看不到 `StampedObservation` 全文——节点目前只把 `msg.obs` 传给 `update`

因此有两条路，选一：

- **A（推荐，接口仍干净）**：把自车量拷进 `ArmorObservation` 的新字段，或
- **B**：扩展 `ITracker::update` 为 `update(obs, sim_t)` / 增加 `update_stamped(StampedObservation)` — 这会改基类，本分支里让 `BaselineTracker` 转调旧 `update`

不要为了传 ω 去 include `sim/kinematics.hpp`。

构造期常量（固定 `chassis_w`、`gimbal_offset`）可以直接从 YAML 注入构造函数，不必走消息。时变量必须走消息。

## 新 ITracker 最小骨架

```cpp
class CompensatedTracker : public ITracker {
public:
  void init(const common::ArmorObservation& obs, double t) override;
  void predict(double dt) override;
  void update(const common::ArmorObservation& obs) override; // 空则 init
  common::TargetEstimate estimate() const override;
  Eigen::Vector3d predict_aim_point(double flight_time) const override;
  bool diverged() const override;
  double last_nis() const override;
  std::string name() const override { return "CompensatedTracker"; }
};
```

`estimate()` 至少填：`x`(≥11)、`center`、`velocity`、`yaw`、`w`、`radius`、`armor_xyza_list`、`last_nis`、`diverged`。
`predict_aim_point` 与 `compute_aim_lead_point` 公式保持一致，除非算法故意改瞄准律（须在提交说明写清）。

接线只改一处：

```cpp
auto tracker = std::make_unique<tracker::CompensatedTracker>(...);
```

## 验收

- 能编过：`cmake --build cmake-build-debug`
- 默认 yaml 跑几秒不发散；终端 `kal-true` 误差与 HUD / `err_*` 曲线同口径
- OmniScope 上 `obs_*` / `true_*` / `kal_*` 仍在；新键可另开
- 3D：Kalman 橙、AIM 金；全部绕云台原点（真值不进 3D，只在曲线 / HUD）
- 相对 main 的 diff 应集中在 tracker 新文件 + 少量接线；若动了 `common/types.hpp`，字段必须有默认值

## 和无界面 benchmark

`experiments/bench.cpp` 是独立定量脚本（不经 OmniScope），直接调 `KinematicsEngine` / `CameraSensor` / `BaselineTracker`。分支上可比对 BASE vs 新 variant；不要把它的补偿逻辑复制进 `main` 的节点路径，除非升格算法。

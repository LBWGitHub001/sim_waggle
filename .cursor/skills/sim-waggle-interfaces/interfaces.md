# 接口明细

头文件 include 风格：包内扁平（`"types.hpp"`）；`app/` 与 `render/` 用根式（`"common/types.hpp"`、`"sim/sim_node.hpp"`、`"tracker/baseline_tracker.hpp"`）。

## 1. `common/` — 两节点唯一耦合

### 1.1 领域类型（`types.hpp`，`namespace common`）

| 类型 | 关键字段 | 坐标系 |
|---|---|---|
| `ChassisState` | `pos, vel, yaw, w` | 世界 |
| `GimbalState` | `pos, vel, yaw, pitch, R_gimbal2world` | 世界；`pos/vel ≡ 0`；杆臂 = `-chassis.pos` |
| `TargetState` | `center, vel, yaw, w, radius, armor_num, armors_pos, armors_yaw` | 世界 |
| `GroundTruth` | `t, chassis, gimbal, target, visible_armor_id` | 世界；装甲位姿按 id 索引，不另存一份 |
| `ArmorObservation` | 见下 | 混系，见 frames |
| `TargetEstimate` | 见下 | 世界系（与 EKF 相同） |
| `StampedObservation` | `obs, sim_t, frame` | SimNode → TrackerNode |
| `TrackerOutput` | `est, sim_t` | TrackerNode → 可视化 |
| `SimulationConfig` | 见 YAML 映射 | — |

`ArmorObservation`：

- `xyz_in_gimbal`：相机系（`R_g2w^T * (armor - gimbal.pos)`，随 yaw 转）
- `xyz_in_world`：世界系（原点=云台，轴向固定）
- `ypd_in_world`：对 `xyz_in_world` 的 yaw/pitch/distance
- `yaw_raw`：装甲板世界 yaw（rad）
- `confidence, name, color, type, width, height`

`TargetEstimate`：

- `x`：状态向量。基线 11 维；**下游假定 `x.size() >= 11`**
- `center = (x0, x2, x4)`，`velocity = (x1, x3, x5)`，`yaw=x6`，`w=x7`，`radius=x8`
- `armor_xyza_list`：每块 `[x, y, z, yaw]`
- `last_nis, diverged`

扩展状态时：保留前 11 维语义，或在本分支同步改 `metrics` / `tracker_node` plot / `main` marker 的 `size() >= 11` 门槛。无论哪种，都要填上述导出字段。

### 1.2 通道（`channel.hpp`）

```
Channel<T>::push(T)           // 永不阻塞；closed 后丢弃
Channel<T>::wait_pop(T&)      // 阻塞；false = 已关闭且排空
Channel<T>::close()
Channel<T>::size()            // 积压；持续 >1 说明 tracker 跟不上

LatestSlot<T>::set(T)
LatestSlot<T>::get(T&) const  // 从未写入 → false
```

### 1.3 评估（`metrics.hpp`）

```
SimulationMetrics::update(gt, est, bullet_speed=25) -> MetricsResult
SimulationMetrics::latest() / reset()
compute_aim_lead_point(est, flight_time) -> Vector3d
  // armor = center + (-r cos yaw, -r sin yaw, 0) + vel * T
```

`MetricsResult`：误差 = Kalman − 真值（世界系）。

- 分量：`err_{x,y,z,vx,vy,vz,yaw,w,r}`
- 范数：`pos_err_norm = |Δcenter|`，`vel_err_norm = |Δvel|`
- 瞄准：`aim_lead_bias = |aim_kal - aim_true|`（同一 `flight_time`、同一落点公式）
- 几何：`armor_err_norm`（拟合板到最近真值板均距）
- 累计：`samples, rmse_pos/vel, mean_*, max_*`

`est.x.size() < 11` 时跳过评估，避免 0 污染。

已知口径缺口（改指标时要知情）：`aim_lead_bias` 只用速度误差，不含位置滞后；NIS 在 EKF 里用后验残差，阈值 0.711 不是 χ²(4) 的 95% 上分位。见 `docs/analysis_and_solutions.md`。

### 1.4 FFT（`fft.hpp`）

`FftAnalyzer(n=512, hop=n/2)`：`push(v, t) → bool` 窗口就绪；`compute(freq, mag, fs)` 单边 Hann 归一化幅度谱。给频域实验用；`tracker_node.cpp` 目前只 include、未接线。

## 2. `sim/` — 物理与观测

### `KinematicsEngine`（`kinematics.hpp`）

```
KinematicsEngine(SimulationConfig)
reset() / step(dt)
ground_truth() const -> const GroundTruth&
set_chassis_w / set_gimbal_offset / set_target_vel
```

积分在辅助惯性系（配置里底盘初值在原点）。`ground_truth()` 已变到世界系（原点=云台，`gimbal.pos/vel=0`）。

内部派生量：

```
r          = Rz(chassis.yaw) * gimbal_offset
gimbal.pos = chassis.pos + r
gimbal.vel = chassis.vel + ω × r
gimbal.yaw = atan2(target.center - gimbal.pos)
armor_i    = center + (-r cos θi, -r sin θi, 0)
可见板     = 法向·(云台-装甲) 最大者
然后整体减 gimbal.pos / gimbal.vel → 对外世界系
```

### `CameraSensor`（`camera_sensor.hpp`）

```
observe(const GroundTruth&) -> ArmorObservation
```

噪声：平面高斯，z 不加噪；yaw 加 `noise_yaw_std`。种子 42。

### `SimNode`（`sim_node.hpp`）

```
struct Topics {
  Channel<StampedObservation>& obs_out;
  LatestSlot<GroundTruth>&     gt_out;
  LatestSlot<StampedObservation>& obs_latest;
};
SimNode(SimulationConfig, Topics)
start() / stop() / join() / finished() / frames_published()
```

循环：先 `publish(0, 0)` → `step(dt)` → `publish(frame*dt, frame)`。`duration>0` 到点退出并 `obs_out.close()`。

## 3. `tracker/` — 滤波

### 具体 tracker（无虚基类）

不要 `ITracker`、不要 `virtual`、不要继承多态。每个分支 `TrackerNode` 持有该分支自己的具体类型。实现必须线程内同步调用（TrackerNode 单线程）。

`main` 上是 `BaselineTracker`。实验分支把 `TrackerNode` 的持有类型换成新类，不要再加基类。

### `BaselineTracker`

```
BaselineTracker(target_radius=0.20, armor_num=4, EkfParams, p0=Zero(11))
```

`update` 在 `target_==nullptr` 时调 `init`。`to_auto_aim_armor` 必须写 `ypr_in_world = (yaw_raw, 0, 0)`，否则 yaw 通道恒 0。

### `auto_aim::Target` / `EkfParams`

```
EkfParams { q_xyz_acc_var=100, q_yaw_acc_var=75, r_yaw=0.04, r_pitch=0.04, r_dist_base=0.015 }
Target(armor, t, radius, armor_num, P0_dig, ekf_params)
predict(dt) / update(armor)
ekf_x() / ekf() / armor_xyza_list() / diverged()
```

观测更新 `update_ypda`：z = `[ypd_yaw, ypd_pitch, ypd_dist, ypr_yaw]`。奇偶板用 `r+l` 与 `zc+h`。

### `tools::ExtendedKalmanFilter`

```
x, P, last_nis, data["nis"|"nees"|...]
predict(F, Q) / predict(F, Q, f)
update(z, H, R, z_subtract) / update(z, H, R, h, z_subtract)
```

约瑟夫形式 `P = (I-KH)P(I-KH)' + KRK'`。`x_add` 默认 `a+b`，Target 对 yaw(`x[6]`) 做 `limit_rad`。

### `auto_aim::Armor`（`armor.hpp`）

原版几何/枚举。仿真只用 `xyz_*`、`ypd_in_world`、`ypr_in_world`、`yaw_raw`。

### `tools::` 数学（`math_tools.hpp`）

`limit_rad`、`xyz2ypd` / `ypd2xyz` 及雅可比、`eulers`、`rotation_matrix`、`delta_time`。

### `TrackerNode`

```
TrackerNode(unique_ptr<BaselineTracker>, Channel<StampedObservation>&, LatestSlot<TrackerOutput>&)
start() / stop() / join() / finished()
```

## 4. `app/` 接线

`main` 创建四条话题、两个节点、一次 `RemoteLogger::init`。可视化循环读三槽齐备才渲染。

```
Channel<StampedObservation> obs_channel;
LatestSlot<GroundTruth> gt_slot;
LatestSlot<StampedObservation> obs_slot;
LatestSlot<TrackerOutput> est_slot;

SimNode sim(cfg.sim, {obs_channel, gt_slot, obs_slot});
TrackerNode tracker(make_unique<...>(...), obs_channel, est_slot);
```

CLI：`sim_waggle [config.yaml]`，默认 `configs/sim_waggle.yaml`。

构建（仓库无 `build.sh`/`run.sh`）：

```
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
./cmake-build-debug/sim_waggle configs/sim_waggle.yaml
```

Host：`thirdparty/OmniScope/host/start.sh`，门户 `http://127.0.0.1:8080`。

## 5. YAML ↔ `SimulationConfig` / `EkfParams`

| YAML | 字段 | 默认 |
|---|---|---|
| `simulation.fps` | `sim.fps` | 50（文件里常为 100） |
| `simulation.duration` | `sim.duration` | 0=无限 |
| `my_robot.chassis_w/vx/vy` | 同名 | 10, 0, 0 |
| `my_robot.gimbal_offset` | `[dx,dy,dz]` m | (0.12,0,0)（文件里常为 0.05） |
| `target_robot.init_pos/vel` | `target_init_pos/target_vel` | (3.5,0,0) / 0 |
| `target_robot.radius/w/armor_num` | 同名 | 0.20 / 4 / 4 |
| `measurement.enable_noise` | | true |
| `measurement.noise_pos_std` | m | 0.005 |
| `measurement.noise_yaw_std` | rad | 0.01 |
| `ekf.q_xyz_acc_var / q_yaw_acc_var` | | 100 / 75 |
| `ekf.r_yaw / r_pitch / r_dist_base` | | 0.04 / 0.04 / 0.015 |
| `ekf.p0` | 11 维对角 | `1,64,1,64,1,64,0.4,400,1e-5,1,1` |
| `remote_logger.*` | `omniscope::RemoteLogger::Config` | 见 yaml |

`bullet_speed` 在 `SimulationConfig`（默认 25）但 YAML 未暴露；要扫弹速就在本分支加字段。

新算法参数：在 YAML 加独立节（如 `algo:`），只在 `load_config` + 该分支构造函数读取。

## 6. 渲染（`render/`，app 私有）

```
CameraDetectorRenderer(width=800, height=600, fov_deg=60)
render(gt, est, obs, metrics) -> cv::Mat
```

针孔、LED 灯条、检测框、EKF 重投影、AIM LEAD。图像经 `logger.plot_image(frame, {{"name","camera_detector"}})` 上报。

## 7. OmniScope C++（只读使用）

```
omniscope::init(cfg) / shutdown()
omniscope::plot({{"key", value}, ...})
omniscope::log / info / warn / error
omniscope::plot_marker / plot_markers
omniscope::plot_image(cv::Mat, meta)
omniscope::poll_json
```

`RemoteLogger::Config`：`control_port, beacon_port, enable_remote/local, log_dir, sender_name, img_width/quality, heartbeat/beacon_interval_ms, head_timeout_ms`。

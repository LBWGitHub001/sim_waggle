# 接口明细

头文件 include 风格：包内扁平（`"types.hpp"`）；`app/` 与 `render/` 用根式（`"common/types.hpp"`、`"sim/sim_node.hpp"`、`"tracker/baseline_tracker.hpp"`）。

## 1. `common/` — 两节点唯一耦合

### 1.1 领域类型（`types.hpp`，`namespace common`）

| 类型 | 关键字段 | 坐标系 |
|---|---|---|
| `ChassisState` | `pos, vel, yaw, w` | odom |
| `GimbalState` | `pos, vel, yaw=0, pitch, R_gimbal2world=I` | odom 原点；轴向与 odom 平行 |
| `CameraState` | `pos, yaw, pitch, R_camera2odom` | 与 gimbal 同原点；`R` = look-at → odom |
| `TargetState` | `center, vel, yaw, w, radius, armor_num, armors_pos, armors_yaw` | odom（装甲由 robot 变来） |
| `GroundTruth` | `t, chassis, gimbal, camera, target, visible_armor_id` | odom 中间件 |
| `ArmorObservation` | 见下 | 工作系 gimbal |
| `TargetEstimate` | 见下 | gimbal |
| `MetricsResult` | `err_*` / rms | gimbal；定义在 `types.hpp` |
| `StampedObservation` | `obs, sim_t, frame, ego_acc` | SimNode → TrackerNode；本分支 `ego_acc` 为 gimbal xy |
| `TrackerOutput` | `est, sim_t, metrics` | TrackerNode → 可视化 / HUD |
| `SimulationConfig` | 见 YAML 映射 | — |

`ArmorObservation`：

- `xyz_in_gimbal`：工作系（`armor_odom - gimbal.pos`，轴向=odom）
- `xyz_in_world`：装甲 odom（中间件，滤波/plot 不用）
- `ypd_in_world`：对 `xyz_in_gimbal` 的 yaw/pitch/distance
- `yaw_raw`：装甲 yaw 在方向不变的 gimbal（= odom yaw）
- `confidence, name, color, type, width, height`

`common/frames.hpp`：`express_target_in_gimbal`、`express_target_in_camera`、`point/vel/yaw_odom_to_gimbal`、`point/yaw_odom_to_camera`、`armor_in_robot`。

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

`MetricsResult`：误差 = Kalman − `gt.target`（调用方须先把 target 变到 gimbal）。由 TrackerNode 更新，塞进 `TrackerOutput.metrics`。

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

积分在 odom（底盘初值在原点）。`ground_truth()` 即 odom。`origin_acc()` 为云台原点惯性加速度。

内部派生量：

```
r          = Rz(chassis.yaw) * gimbal_offset
gimbal.pos = chassis.pos + r
gimbal.vel = chassis.vel + ω × r
gimbal.yaw = 0，R = I
camera.pos = gimbal.pos
camera.yaw = atan2(target.center - gimbal.pos)
p_i_robot  = (-r cos(2π i/N), -r sin(...), 0)
armor_i    = center + Rz(robot.yaw) * p_i_robot
可见板     = 法向·(云台-装甲) 最大者
```

### `CameraSensor`（`camera_sensor.hpp`）

```
observe(const GroundTruth&) -> ArmorObservation
```

噪声：平面高斯，z 不加噪；yaw 加 `noise_yaw_std`。种子 42。加速度 IMU 噪声在 `SimNode`（种子 43，与相机独立）：`u = (-a_x,-a_y,0) + N(0, noise_acc_std²)`（`enable_noise` 时）。

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

循环：先 `publish(0, 0)` → `step(dt)` → `publish(frame*dt, frame)`。`duration>0` 到点退出并 `obs_out.close()`。`publish` 同时发 gimbal 系 `obs_*`/`true_*`、带噪 `ego_acc_*`、`plot_tfs`、`my_robot.*`/`robot.armor` Marker。

## 3. `tracker/` — 滤波

### 具体 tracker（无虚基类）

不要 `ITracker`、不要 `virtual`、不要继承多态。每个分支 `TrackerNode` 持有该分支自己的具体类型。实现必须线程内同步调用（TrackerNode 单线程）。

`main` 上是 `BaselineTracker`。实验分支把 `TrackerNode` 的持有类型换成新类，不要再加基类。

### `BaselineTracker`

```
BaselineTracker(target_radius=0.20, armor_num=4, EkfParams, p0=Zero(11))
```

`update` 在 `target_==nullptr` 时调 `init`。`to_auto_aim_armor` 必须写 `ypr_in_world = (yaw_raw, 0, 0)`，否则 yaw 通道恒 0。

### `EgoControlTracker`（本分支）

```
EgoControlTracker(target_radius=0.20, armor_num=4, EkfParams, p0=Zero(11))
predict(dt)                 // Target::predict：只 Fx
control(ego_acc)            // Target::control：Bu + B M Bᵀ，gimbal xy，z 忽略
```

过程 `x = Fx + B(u+m) + w`。`u` 是 SimNode 发出的带噪加速度（真值 `(-a_x,-a_y)` 加 `N(0,σ²)`）。`B` 为 11×2（`½dt²/dt` 进 xc/vx 与 yc/vy）。`M = q_u_acc_var I₂`。xy 不再用 `q_xyz_acc_var`。推导见仓库 README 第 3 节。

### `auto_aim::Target` / `EkfParams`

```
EkfParams { q_xyz_acc_var=100, q_yaw_acc_var=75, q_u_acc_var=1, r_yaw=0.04, r_pitch=0.04, r_dist_base=0.015 }
Target(armor, t, radius, armor_num, P0_dig, ekf_params)
predict(dt) / control(acc) / update(armor)
ekf_x() / ekf() / armor_xyza_list() / diverged()
```

`predict` 只建 F；`control` 用 `last_dt_` 建 B；`update_ypda` 组 `Q_{z,ψ}`（首帧 `last_dt_=0` 则 Q=0）再 `ekf_.update(..., Q)`。z = `[ypd_yaw, ypd_pitch, ypd_dist, ypr_yaw]`。奇偶板用 `r+l` 与 `zc+h`。

### `tools::ExtendedKalmanFilter`

```
x, P, last_nis, data["nis"|"nees"|...]
predict(F) / predict(F, f)          // x=Fx, P=FPFᵀ；Q 不进 predict
control(B, u, M)                    // x+=Bu, P+=B M Bᵀ
update(z, H, R, Q, ...)             // P+=Q，再约瑟夫量测更新
```

先验协方差等价 `P⁻ = FPFᵀ + BMBᵀ + Q`。约瑟夫形式 `P = (I-KH)P(I-KH)' + KRK'`。`x_add` 默认 `a+b`，Target 对 yaw(`x[6]`) 做 `limit_rad`。

### `auto_aim::Armor`（`armor.hpp`）

原版几何/枚举。仿真只用 `xyz_*`、`ypd_in_world`、`ypr_in_world`、`yaw_raw`。

### `tools::` 数学（`math_tools.hpp`）

`limit_rad`、`xyz2ypd` / `ypd2xyz` 及雅可比、`eulers`、`rotation_matrix`、`delta_time`。

### `TrackerNode`

```
TrackerNode(unique_ptr<EgoControlTracker>, Channel<StampedObservation>&,
            LatestSlot<TrackerOutput>&, LatestSlot<GroundTruth>&, bullet_speed=25)
start() / stop() / join() / finished()
```

每帧：首帧只 `update`；之后 `predict(dt)` → `control(msg.ego_acc)` → `update(obs)`。`estimate()` 后 `express_target_in_gimbal` + `SimulationMetrics::update`，写 `TrackerOutput.metrics`，发 `kal_*`/`err_*` 与 Kalman Marker。

## 4. `app/` 接线

`main` 创建四条话题、两个节点、一次 `RemoteLogger::init`。可视化循环读三槽齐备才 `plot_image`；**不** `plot` / `plot_markers` / `plot_tfs`。

```
Channel<StampedObservation> obs_channel;
LatestSlot<GroundTruth> gt_slot;
LatestSlot<StampedObservation> obs_slot;
LatestSlot<TrackerOutput> est_slot;

SimNode sim(cfg.sim, {obs_channel, gt_slot, obs_slot});
TrackerNode tracker(make_unique<...>(...), obs_channel, est_slot, gt_slot, bullet_speed);
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
| `measurement.noise_acc_std` | 仿真 IMU xy 噪声 σ (m/s²) | 1.0 |
| `ekf.q_xyz_acc_var / q_yaw_acc_var` | z / yaw 未建模加速度 | 100 / 75 |
| `ekf.q_u_acc_var` | 滤波 M=σ² I₂，经 B 进 P | 1.0 |
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

针孔、LED 灯条、检测框、EKF 重投影、AIM LEAD。HUD 在 camera：`cam_pos=0`，敌车 `express_target_in_camera`；Kalman 从 gimbal 旋到 camera。图像经 `logger.plot_image(frame, {{"name","camera_detector"}})` 上报。

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

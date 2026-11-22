# 接口明细

头文件 include 风格：包内扁平（`"types.hpp"`）；`app/` 用根式（`"common/types.hpp"`、`"sim/sim_node.hpp"`、`"tracker/tracker_node.hpp"`、`"aimer/aimer_node.hpp"`）。

## 1. `common/` — 三节点唯一耦合（头文件库）

### 1.1 领域类型（`types.hpp`，`namespace common`）

| 类型 | 关键字段 | 坐标系 |
|---|---|---|
| `ChassisState` | `pos, vel, yaw, w` | odom |
| `GimbalState` | `pos, vel, yaw=0, pitch, R_gimbal2world=I` | odom 原点；轴向与 odom 平行 |
| `CameraState` | `pos, yaw, pitch, R_camera2odom` | 与 gimbal 同原点；`R` = look-at → odom |
| `TargetState` | `center, vel, yaw, w, radius, l, h, armor_num, armors_pos, armors_yaw` | odom（装甲由 robot 变来） |
| `GroundTruth` | `t, chassis, gimbal, camera, target, visible_armor_id` | odom 中间件 |
| `ArmorObservation` | 见下 | 工作系 gimbal |
| `TargetEstimate` | 见下 | gimbal |
| `MetricsResult` | `err_*` / rms | gimbal；定义在 `types.hpp` |
| `StampedObservation` | `obs, sim_t, frame, stamp_ns`（`exp/ego-acc` 另有 `ego_acc`） | SimNode → TrackerNode |
| `GimbalCommand` | `valid, target_yaw/pitch, yaw/pitch, *vel/*acc, fly_time, aim_xyz` | gimbal；无 `shoot` |
| `AimerParams` | 偏移、coming/leaving、delay | — |
| `TrackerOutput` | `est, sim_t, stamp_ns, metrics`（**不含 cmd**） | TrackerNode → AimerNode |
| `SimulationConfig` | 见 YAML 映射 | — |

`ArmorObservation`：

- `xyz_in_gimbal`：工作系（`armor_odom - gimbal.pos`，轴向=odom）
- `xyz_in_world`：装甲 odom（中间件，滤波/plot 不用）
- `ypd_in_world`：对 `xyz_in_gimbal` 的 yaw/pitch/distance
- `yaw_raw`：装甲 yaw 在方向不变的 gimbal（= odom yaw）
- `armor_id`：仿真 `visible_armor_id`，EKF 直接用，不匹配
- `confidence, name, color, type, width, height`

`common/frames.hpp`：`express_target_in_gimbal`、`express_target_in_camera`、`point/vel/yaw_odom_to_gimbal`、`point/yaw_odom_to_camera`、`armor_in_robot(i,n,r,l=0,h=0)`。四装甲：id 0/2 用 `r,z=0`；id 1/3 用 `r+l,z=h`。

`TargetEstimate`：

- `x`：状态向量。基线 11 维；**下游假定 `x.size() >= 11`**
- `center = (x0, x2, x4)`，`velocity = (x1, x3, x5)`，`yaw=x6`，`w=x7`，`radius=x8`，`l=x9`，`h=x10`
- `jumped`：main 上恒 `true`（Aimer coming/leaving）
- `armor_num, armor_xyza_list`：每块 `[x, y, z, yaw]`
- `last_nis, diverged`

### 1.2 通道（`channel.hpp`）

```
Channel<T>::push(T)           // 永不阻塞；closed 后丢弃
Channel<T>::wait_pop(T&)      // 阻塞；false = 已关闭且排空
Channel<T>::close()
Channel<T>::size()

LatestSlot<T>::set(T)
LatestSlot<T>::get(T&) const  // 从未写入 → false
```

### 1.3 评估（`tracker/metrics.hpp`）

```
SimulationMetrics::update(gt, est, bullet_speed=25) -> MetricsResult
SimulationMetrics::latest() / reset()
compute_aim_lead_point(est, flight_time) -> Vector3d
```

`MetricsResult`：误差 = Kalman − `gt.target`（调用方须先把 target 变到 gimbal）。由 TrackerNode 更新，塞进 `TrackerOutput.metrics`。

`est.x.size() < 11` 时跳过评估。

### 1.4 弹道（`aimer/`：`trajectory.hpp` / `aim.hpp`）

- `BulletTrajectory(v0, d, h)`：无阻力，`pitch` 抬头为正
- `propagate_estimate(est, dt)`：11 维 CV + `h_armor_xyz` 重建装甲
- `Aimer::aim`：coming/leaving 选板 + 飞行时间迭代 → 无约束 `cmd.yaw/pitch`（与 `target_*` 相同）。无 MPC、无 `fire`

## 2. `sim/` — 物理与观测

### `KinematicsEngine`（`kinematics.hpp`）

```
KinematicsEngine(SimulationConfig)
reset() / step(dt)
ground_truth() const -> const GroundTruth&
```

积分在 odom。`origin_acc()` 为云台原点惯性加速度。可见板 = 法向·(云台-装甲) 最大者。

### `SimNode`（`sim_node.hpp`）

观测生成在 `SimNode::observe`（原 CameraSensor 并入）。RNG 种子 42。平面高斯，z 不加噪。`obs.armor_id = gt.visible_armor_id`。

```
struct Topics {
  Channel<StampedObservation>& obs_out;
  LatestSlot<GroundTruth>&     gt_out;
};
SimNode(SimulationConfig, Topics)
start() / stop() / join() / finished() / frames_published()
```

循环：先 `publish(0, 0)` → `step(dt)` → `publish(frame*dt, frame)`。`duration>0` 到点退出并 `obs_out.close()`。`publish` 打一次 `stamp_ns = omniscope::now_ns()`，随观测发出；本帧 `obs_*`/`true_*`、`plot_tfs`、`my_robot.*`/`robot.armor` 都带这个 `t_mark`。

## 3. `tracker/` — 滤波

### `tracker::Target` / `EkfParams`

```
EkfParams { q_xyz_acc_var=100, q_yaw_acc_var=75, r_yaw=0.04, r_pitch=0.04, r_dist_base=0.015 }
Target(radius, armor_num, P0_dig, ekf_params)
init(obs) / predict(dt) / update(obs)
estimate() -> TargetEstimate
```

`init`：车心 = 装甲位置 + r*(cos yaw, sin yaw, 0)，`x0 = {cx,0,cy,0,cz,0,yaw,0,r,0,0}`。
`update`：`id = obs.armor_id`；`R = diag(r_yaw, r_pitch, r_dist_base, r_yaw)`。无切板排序、无跳变计数、无前哨站钉 ω、无自适应 R。

### `tools::ExtendedKalmanFilter`

```
predict(F, Q) / predict(F, Q, f)
update(z, H, R, z_subtract) / update(z, H, R, h, z_subtract)
```

约瑟夫形式 `P = (I-KH)P(I-KH)' + KRK'`。Target 对 yaw(`x[6]`) 做 `limit_rad`。

### `TrackerNode`

```
TrackerNode(radius, armor_num, EkfParams, p0,
            Channel<StampedObservation>&, Channel<TrackerOutput>&,
            LatestSlot<GroundTruth>&, bullet_speed=25)
start() / stop() / join() / finished()
```

每帧 `estimate()` 后 `express_target_in_gimbal` + `SimulationMetrics::update`，把 `msg.stamp_ns` 拷进 `TrackerOutput`，发带 `t_mark` 的 `kal_*`/`err_*` 与 Kalman Marker。禁止再 `now_ns()`。退出时 `est_out.close()`。

## 4. `aimer/` — 火控

### `AimerNode`

```
AimerNode(AimerParams,
          Channel<TrackerOutput>&, LatestSlot<GroundTruth>&,
          LatestSlot<GimbalCommand>&, bullet_speed=25)
```

同一套 `Aimer::aim(state)`：Kalman `msg.est` → `cmd_est`（写 `cmd_out`）；`estimate_from_ground_truth(gt)` → `cmd_true`。两套 Aimer 实例，选板 lock 互不污染。发带 `msg.stamp_ns` 的 `cmd_*`、`true_aim_*`、`aim.lead` Marker。禁止再 `now_ns()`。

## 5. `app/` 接线

`main` 创建话题、三个节点、一次 `RemoteLogger::init`。无渲染循环。SIGINT：`sim.stop(); join 三个`。

```
Channel<StampedObservation> obs_channel;
LatestSlot<GroundTruth> gt_slot;
Channel<TrackerOutput> est_channel;
LatestSlot<GimbalCommand> cmd_slot;

SimNode sim(cfg.sim, {obs_channel, gt_slot});
TrackerNode tracker(radius, armor_num, ekf, p0, obs_channel, est_channel, gt_slot, bullet_speed);
AimerNode aimer(cfg.sim.aimer, est_channel, gt_slot, cmd_slot, bullet_speed);
```

CLI：`sim_waggle [config.yaml]`，默认 `configs/sim_waggle.yaml`。

```
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
./cmake-build-debug/sim_waggle configs/sim_waggle.yaml
```

Host：`thirdparty/OmniScope/host/start.sh`，门户 `http://127.0.0.1:8080`。

## 6. YAML ↔ `SimulationConfig` / `EkfParams`

| YAML | 字段 | 默认 |
|---|---|---|
| `simulation.fps` | `sim.fps` | 50（文件里常为 100） |
| `simulation.duration` | `sim.duration` | 0=无限 |
| `my_robot.chassis_w/vx/vy` | 同名 | 10, 0, 0 |
| `my_robot.gimbal_offset` | `[dx,dy,dz]` m | (0.12,0,0)（文件里常为 0.05） |
| `target_robot.init_pos/vel` | `target_init_pos/target_vel` | (3.5,0,0) / 0 |
| `target_robot.radius/l/h/w/armor_num` | 对侧 r；邻侧 r+l、z+h | 0.20 / 0.05 / 0.05 / 4 / 4 |
| `simulation.bullet_speed` | `sim.bullet_speed` | 25 |
| `aimer.yaw_offset/pitch_offset` | 度→rad | 0 |
| `aimer.coming_angle/leaving_angle` | 度→rad | 55 / 20 |
| `aimer.delay_time` | s | 0 |
| `measurement.enable_noise` | | true |
| `measurement.noise_pos_std` | m | 0.005 |
| `measurement.noise_yaw_std` | rad | 0.01 |
| `ekf.q_xyz_acc_var / q_yaw_acc_var` | | 100 / 75 |
| `ekf.r_yaw / r_pitch / r_dist_base` | | 0.04 / 0.04 / 0.015 |
| `ekf.p0` | 11 维对角 | `1,64,1,64,1,64,0.4,400,1e-5,1,1` |
| `remote_logger.*` | `omniscope::RemoteLogger::Config` | 见 yaml |

新算法参数：在 YAML 加独立节（如 `algo:`），只在 `load_config` + 该分支读取。

## 7. OmniScope C++（只读使用）

未经用户点名 **禁止** 改 SDK / Host。本仓库只用：

```
omniscope::init(cfg) / shutdown()
omniscope::plot({{"key", value}, ...})
omniscope::log / info / warn / error
omniscope::plot_marker / plot_markers
omniscope::plot_tfs
```

main 禁止 `plot` / `plot_image`。

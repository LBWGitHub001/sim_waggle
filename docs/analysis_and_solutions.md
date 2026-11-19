# 底盘偏置公转对自瞄 Kalman 的影响：诊断、定量结果与解决方案

> 分析对象：/home/lbw/Project/RM2027/sim_waggle
> 方法：读代码 + 独立无界面 benchmark（experiments/bench.cpp，复用 sim::KinematicsEngine / CameraSensor / tracker::BaselineTracker）+ 现有工程实测
> 结论对应当前仓库代码，未修改任何原有源文件

---

## 0. 结论（TL;DR）

1. **根因不是 Q/R 调参，而是坐标系语义不一致。** 观测 'xyz_in_world' 实际是"装甲板相对云台的位置"，少了 '+ p_gimbal'；EKF 却把它当世界绝对位置，超前瞄准又把 EKF 速度当目标绝对速度用。
   代码位置：sim/camera_sensor.cpp:40-47、tracker/target.cpp:39、tracker/baseline_tracker.cpp:89-106。

2. **真正的误差主体是"旋转的杆臂位置"，不是"伪速度"本身。** 底盘自转把固定偏置 d 旋转成 p_g(t)=R_z(θ)d，观测里少了它；伪速度只是它的导数。
   |p_g| = r，|v_g| = ω r；ω=12 rad/s、r=0.1 m 时伪速度 1.2 m/s。
   当滤波器能跟上这个旋转时，瞄准横向误差的相位 RMS 约为 r/√2（r=0.1 m 时 7.1 cm），**任何 Q/R 调参都消不掉这个几何项**。

3. **默认配置把问题掩盖了。** r_yaw=0.04 rad 是传感器 noise_yaw_std=0.01 rad 的 4 倍，EKF 带宽只有约 2–4 rad/s，12 rad/s 的扰动被"平均"掉了：实测速度误差仅 0.05 m/s、横向瞄准误差 2.3 cm，看起来"没事"。
   但把 R 调到与传感器一致（0.01）后，同样 r=0.05 m 的瞄准误差变成 3.6 cm；r=0.1 m 时 7.1 cm；r=0.2 m 时 14.4 cm。

4. **最坏工况不是 12 rad/s，而是 2–4 rad/s 的慢晃。** 在默认 R 下、r=0.1 m：ω=4 rad/s 时横向误差 11.1 cm（伪速度只有 0.4 m/s），ω=12 rad/s 时 4.6 cm（伪速度 1.2 m/s）。
   **不能用 |v_ghost| 或 vel_err*T 衡量危害，必须用真实瞄准落点误差。**

5. **唯一稳健的修法：在观测/过程模型里把自车运动显式建模。**
   - **S1（首选）**：观测变换到带平移的世界系：xyz_world = R_g2w * xyz_gimbal + p_gimbal；EKF 估计绝对目标运动，目标在滤波系里几乎静止，误差降到噪声地板（0.1–0.3 cm）。
   - **S2（最小改动兜底）**：没有 TF 平移时，把云台世界速度 v_g 作为已知输入塞进 EKF：p_rel' = v_target - v_g，位置仍是相对、速度变成绝对；数学上与 S1 等价（差一个已知时变平移），只需要陀螺+底盘里程计+偏置。
   - **S3**：完全没有自车状态时，用已知 ω 的谐波/偏置状态增广在线估计杆臂，再扣除；复杂、可观测性差，仅作兜底。
   - **S4**：调 Q/R、加 notch 只能把误差在"跟得上→滞后"之间挪，不能消除；实测把 Q 从 1 调到 1000，瞄准误差从 2.1 cm 单调升到 13.8 cm。
   - **S5**：机械上减小水平偏置 r（误差与 r 成正比）；战术上避开 2–4 rad/s 的慢晃窗口。

6. **瞄准级"事后补偿"不可行（实测反证）。** 在瞄准点加 v_g*T 误差 16.5 cm；加精确云台位移误差 14.0 cm；都比不补偿的 4.6–7.1 cm 更差。原因：CV-EKF 在 1.9 Hz 处的速度估计相位滞后、幅值被压缩，拿它做前馈会"补偿过头"。必须把自车运动从"待估计量"变成"已知量"。

7. **测量与可视化本身也在混帧，会误导结论：**
   - common/metrics.cpp:17-30 用相对估计减绝对真值，aim_lead_bias = vel_err*T 漏掉位置滞后项（实测真实瞄准误差 4.6 cm，而 aim_bias 只有 1.5 cm）。
   - tracker/extended_kalman_filter.cpp:59-66 的 NIS 用的是**后验**残差和后验 P；阈值 0.711 是 χ²(4) 的 5% 下分位，不是 95% 上分位（应为 9.488）。当前 NIS 遥测没有统计意义。
   - render/camera_detector_renderer.cpp:193,325-334,363-366 用真实云台位置 cam_pos=gt.gimbal.pos 去投影**相对坐标**的 est.center；3D marker 的 aim.lead 射线从绝对 gt.gimbal.pos 连到相对 aim_lead_pt。这会造成 5–10 cm 的固定视觉偏差，和 Kalman 误差混在一起。

---

## 1. 代码链路与根因

### 1.1 数据流

SimNode（sim/sim_node.cpp）50/100 Hz：

- KinematicsEngine::update_gimbal()（sim/kinematics.cpp:84-96）：

      r_world    = R_z(chassis.yaw) * gimbal_offset;
      gimbal.pos = chassis.pos + r_world;
      gimbal.vel = chassis.vel + ω × r_world;
      gimbal.yaw = atan2(target.center - gimbal.pos);

- CameraSensor::observe()（sim/camera_sensor.cpp:40-59）：

      r_g2a         = armor_pos - gimbal.pos;      // 真实的"云台→装甲"矢量
      xyz_in_gimbal = R_g2w^T * r_g2a;             // 相机系测量
      xyz_in_world  = r_g2a;                       // ★ 少了 + gimbal.pos
      ypd_in_world  = xyz2ypd(xyz_in_world);
      yaw_raw       = armor_yaw;                   // 世界系装甲 yaw

- TrackerNode（tracker/tracker_node.cpp:37-46）：每帧 predict(sim_t-last_t) + update(obs)；首帧 update 内部 init。

### 1.2 观测是"云台相对位置"，不是世界位置

xyz_in_world = armor - p_g 等价于把世界原点平移到云台，但**轴向仍是世界系**（R*R^T=I）。所以它是"非惯性原点、世界轴向"的相对坐标。
在真实 auto_aim 管线里，这一步是 solver 的 R_gimbal2world * xyz_in_gimbal + t_gimbal2world；本项目故意把 t 置零来复现"TF 平移缺失/为 0"的失效模式。

### 1.3 EKF 状态的实际语义

Target 初始化（tracker/target.cpp:39）：

    center_x = xyz_in_world[0] + r*cos(ypr[0]);
    x0 = {center_x, 0, center_y, 0, center_z, 0, yaw, 0, r, 0, 0};

因为 xyz_in_world 是相对的，x[0],x[2],x[4] 也是相对的；x[1],x[3],x[5] 在模型 p'=v 下自然成为**相对速度** v_target - v_g（如果滤波器跟得上）。
但 predict_aim_point()（tracker/baseline_tracker.cpp:89-106）返回：

    armor_pos = center + (-r cos(yaw), -r sin(yaw), 0);
    return armor_pos + vel * flight_time;

main/render 把它当世界点用。**相对位置 + 相对速度被当成绝对位置 + 绝对速度**，这就是混帧。

### 1.4 误差闭式

设目标静止 v_t=0，云台无平移，杆臂 p_g(t)=R_z(θ_c)d，r=|d_xy|，ω=θ̇_c：

- 观测：z(t) = p_t - p_g(t) + noise。
- EKF 若完全跟上：p_est = p_t - p_g，v_est = -v_g。
- 瞄准点：aim = p_est + v_est*T = p_t - p_g - v_g*T。
- 期望：p_t（目标静止）。
- **误差：-p_g - v_g*T**，其中 |v_g| = ω r，v_g ⟂ p_g。

横向（垂直于视线）分量：

- 杆臂项 -p_g 的横向 RMS（相位均匀）≈ r/√2；
- 速度项 -v_g*T 的横向 RMS ≈ ω r T/√2；
- 两者随相位变化，最大可达 r√(1+(ωT)²)。

ω=12、r=0.1、T=0.14 s：ωT=1.68，几何最大 r√(1+2.82)=0.196 m，相位 RMS 0.138 m。
实测默认 R 下只有 7.1 cm，是因为 EKF 没跟上这个 1.9 Hz 的旋转，把一部分杆臂"平均"掉了；但代价是位置滞后（p_abs 6 cm），换一个 ω 或 R，误差就移到别处。

**要点：v_g 只是 p_g 的导数；"伪速度"是症状，旋转杆臂的位置误差是病根。**

### 1.5 为什么默认配置看起来"没事"

- 传感器噪声：noise_yaw_std=0.01 rad（3.5 m 处 3.5 cm）。
- EKF 假设噪声：r_yaw=0.04 rad（3.5 m 处 14 cm），是前者的 4 倍。
- 结果：滤波器带宽约 2–4 rad/s，ω=12 rad/s 的扰动被低通平均；实测 v_abs=0.104 m/s（真值 1.2 m/s），lead_lat=4.6 cm（r=0.1）。
- README 里"1.2 m/s 疯狂公转的 kalman.vel 箭头"在当前 offset=5cm、r_yaw=0.04 下**不会出现**：估计速度误差只有约 0.05 m/s。README 与 sim_waggle.yaml 的 offset/频率描述也对不上（README 说 50 Hz，config 是 100 Hz；README 的 1.2 m/s 需要 offset≈0.1 m 且滤波器带宽足够高）。

### 1.6 为什么调 Q 不能解决

Q 增大 → 滤波器跟得上旋转杆臂 → 相对速度估计变大 → v_abs 误差和瞄准误差都变大（实测 Q=1:2.1 cm → Q=1000:13.8 cm）。
Q 减小 → 滤波器更慢 → 把 12 rad/s 平均掉 → 12 rad/s 下误差变小，但 2–4 rad/s 的慢晃误差更大（默认 R、r=0.1、ω=4 时 11.1 cm），且真实目标机动会滞后。
这是"跟踪带宽 vs 扰动抑制"的零和，不是调参能消除的系统误差。

### 1.7 为什么瞄准级补偿不行（实测）

以 r=0.1、ω=12、r_yaw=0.01 为例（横向瞄准误差，越小越好）：

| 方案 | 做法 | lead_lat |
|---|---|---|
| BASE | 不补偿 | 7.10 cm |
| LEAD_NAIVE | aim += v_g(t0)*T | 16.48 cm（r_yaw=0.04 配置） |
| LEAD_DISP | aim += p_g(t0+T)-p_g(t0)（漏了 +p_g(t0)） | 17.31 cm |
| LEAD_FULL | aim += p_g(t0) + (p_g(t0+T)-p_g(t0)) | 14.01 cm |
| FRAME | 观测加平移 | **0.15 cm** |

原因：LEAD_FULL 的残差是 (v_est_rel - v_true_rel)*T；CV-EKF 在 1.9 Hz 的速度估计相位滞后、幅值被压缩（v_rel_rms≈1.34 m/s，真值 RMS 0.85 m/s，基本不相关），越补越糟。
**必须让滤波器"知道"自车运动，而不是事后加。**

---

## 2. 定量结果

运行方式见第 6 节。所有指标为稳态（跳过前 1 s）RMS；lead_lat 是预测中心落点相对真实中心落点的**横向**误差，T=range/25；ghost_mean 是真实 |v_g| 均值。

### 2.1 偏置 r 扫描（ω=12，r_yaw=0.01，静止目标，噪声开）

| r (m) | ghost (m/s) | BASE v_abs | BASE lead_lat | FRAME lead_lat |
|---|---|---|---|---|
| 0.00 | 0.00 | 0.004 | 0.16 cm | 0.16 cm |
| 0.02 | 0.24 | 0.037 | 1.45 cm | 0.16 cm |
| 0.05 | 0.60 | 0.092 | 3.57 cm | 0.16 cm |
| 0.10 | 1.20 | 0.183 | 7.10 cm | 0.15 cm |
| 0.20 | 2.40 | 0.358 | 14.38 cm | 0.15 cm |

### 2.2 自转 ω 扫描（r=0.1，**项目默认 r_yaw=0.04**，静止目标）

| ω (rad/s) | ghost (m/s) | BASE p_abs | BASE v_abs | BASE lead_lat | FRAME lead_lat |
|---|---|---|---|---|---|
| 0 | 0.0 | 0.10 | 0.003 | 0.13 cm(注) | 0.13 cm |
| 2 | 0.2 | 0.12 | 0.187 | 9.53 cm | 0.13 cm |
| 4 | 0.4 | 0.11 | 0.238 | **11.09 cm** | 0.13 cm |
| 8 | 0.8 | 0.06 | 0.151 | 6.75 cm | 0.13 cm |
| 12 | 1.2 | 0.04 | 0.104 | 4.60 cm | 0.13 cm |
| 20 | 2.0 | 0.02 | 0.062 | 2.67 cm | 0.13 cm |

(注) ω=0 时偏置方向沿视线，横向误差为 0；若目标不在偏置方向轴线上，静止偏置也会给出最大 r 的横向误差。

**最坏点 4 rad/s，不是 12 rad/s。** 因为 4 rad/s 落在 EKF 带宽附近：位置跟得上但相位滞后约 45°，杆臂几乎原样转进瞄准误差；12 rad/s 被平均掉一部分。

### 2.3 R_yaw 与 Q 的影响（r=0.1，ω=12）

| r_yaw | BASE v_abs | BASE lead_lat | FRAME lead_lat |
|---|---|---|---|
| 0.04（默认） | 0.104 | 4.60 cm | 0.13 cm |
| 0.01（匹配传感器） | 0.183 | 7.10 cm | 0.15 cm |

| Q_xyz | BASE lead_lat（r_yaw=0.01） |
|---|---|
| 1 | 2.13 cm |
| 10 | 3.50 cm |
| 100 | 7.10 cm |
| 1000 | 13.75 cm |

### 2.4 默认工程配置（r=0.05，ω=12，r_yaw=0.04，噪声开，目标 w=7）

| 指标 | BASE | FRAME |
|---|---|---|
| v_abs RMS | 0.052 m/s | 0.003 m/s |
| p_abs RMS | 0.02 m | 0.00 m |
| lead_lat | 2.31 cm | 0.13 cm |

结论：**当前默认配置的测试灵敏度不足以暴露问题**，尤其把 r_yaw 设成传感器噪声的 4 倍，等于人为把滤波器调钝。

---

## 3. 解决方案

按优先级排序。S1/S2 是"治病"，S3 是"没有自车状态时的兜底"，S4 只是"缓解"，S5 是"机械/战术"。

### S1（首选）：观测补上云台平移，统一到世界系

**原理**：让 EKF 估计绝对目标运动；静止目标在滤波系里就是静止，1.9 Hz 扰动不再进入状态估计。
**预期**：上表 FRAME 列，瞄准误差 0.1–0.3 cm（噪声地板），与 r、ω、Q/R 基本无关。

仿真侧（sim/camera_sensor.cpp）：

    const Eigen::Vector3d r_g2a = armor_pos - gt.gimbal.pos;

    // 噪声加在相机系，再变换到世界系（更符合真实链路）
    Eigen::Vector3d z_gimbal = gt.gimbal.R_gimbal2world.transpose() * r_g2a;
    if (cfg_.enable_noise) {
      z_gimbal += Eigen::Vector3d(dist_pos_(rng_), dist_pos_(rng_), 0.0);
    }
    obs.xyz_in_gimbal = z_gimbal;
    obs.xyz_in_world  = gt.gimbal.R_gimbal2world * z_gimbal + gt.gimbal.pos; // ★ + t
    obs.ypd_in_world  = xyz2ypd(obs.xyz_in_world);

真实管线（solver 侧）：

    // t_gimbal2world 必须是"图像曝光时刻"的云台/相机光心平移
    armor.xyz_in_world = R_gimbal2world * armor.xyz_in_gimbal + t_gimbal2world;
    armor.ypd_in_world = xyz2ypd(armor.xyz_in_world);

配套：

- predict_aim_point() 返回世界点；火控/渲染用 aim - p_gimbal(t0) 转到云台系，或直接用世界点。
- 渲染/3D marker 必须与 est 同帧（见 4.3）。
- 代价：需要图像时刻的 TF 平移。若 odom 缓慢漂移，目标绝对位置一起漂，但"相对位置 + 绝对速度"的瞄准几何不变，影响很小。

**验证**：本项目 benchmark 的 FRAME 变体就是 S1 的上界（直接用真值 p_g）；实际误差 = TF 平移误差 + 时间同步误差。

### S2（最小改动兜底）：把云台速度作为 EKF 已知输入

**原理**：状态语义改成"相对位置 + 绝对速度"：

    p_rel = p_target - p_gimbal
    p_rel' = v_target - v_gimbal
    v_target' = 0（CV）

测量仍是相对位置；v_gimbal 是已知输入，不需要估计。
**与 S1 的关系**：p_rel = p_abs - p_g(t) 是已知时变平移，过程模型/协方差做同样变换，二者对 v_target 和瞄准点的估计是等价的；FRAME 的数值可直接作为 S2 的预期。

改动点：

1) common/types.hpp 的消息加字段：

    struct StampedObservation {
      ArmorObservation obs;
      double sim_t = 0.0;
      uint64_t frame = 0;
      Eigen::Vector3d v_gimbal_world = Eigen::Vector3d::Zero(); // 新增：图像时刻云台/相机光心世界速度
    };

2) sim/sim_node.cpp::publish()：

    common::StampedObservation msg{sensor_.observe(gt), sim_t, frame};
    msg.v_gimbal_world = gt.gimbal.vel;   // 真实系统里来自陀螺+底盘里程计+杆臂

3) tracker/itracker.hpp：

    virtual void predict(double dt, const Eigen::Vector3d & v_ego) = 0;
    // 保留旧接口，默认零输入，便于 A/B：
    virtual void predict(double dt) { predict(dt, Eigen::Vector3d::Zero()); }

4) tracker/target.cpp 的 predict：

    void Target::predict(double dt, const Eigen::Vector3d & v_ego)
    {
      // ... F、Q 与原版完全一致 ...
      auto f = [&](const Eigen::VectorXd & x) -> Eigen::VectorXd {
        Eigen::VectorXd xp = F * x;
        // x[0,2,4] 是相对位置；x[1,3,5] 是目标绝对速度
        xp[0] -= v_ego.x() * dt;
        xp[2] -= v_ego.y() * dt;
        xp[4] -= v_ego.z() * dt;
        xp[6] = tools::limit_rad(xp[6]);
        return xp;
      };
      ekf_.predict(F, Q, f);
    }

5) tracker/tracker_node.cpp：

    tracker_->predict(msg.sim_t - last_t, msg.v_gimbal_world);

**注意**：

- predict_aim_point() 返回的是"相对 t0 时刻云台的瞄准点"：armor_rel + v_target_abs*T，这正好是火控需要的发射矢量；若要世界点再加 p_gimbal(t0)。
- 若 v_ego 有噪声，在 P 的位置块加 B Σ_ego B'，B = -dt * [e_x e_y e_z]。陀螺/里程计噪声通常 < 0.05 m/s，对应位置误差 < 0.5 mm/step，可忽略。
- 不要同时对观测再加平移，否则会重复补偿。
- v_ego 要取**相机光心**速度，不只是云台旋转中心：v_cam = v_chassis + ω_chassis × r_chassis→gimbal + ω_gimbal × r_gimbal→camera。本项目相机光心=云台中心，所以最后一项为 0；真实车如果相机前置，必须加。

### S3（兜底）：谐波/杆臂状态增广（没有自车速度时）

如果拿不到 v_g，但知道自转频率 ω（陀螺可测）：

- 扰动在相对系里是 -R_z(θ0+ωt)d，每个轴都是同频正弦；
- 增广 2–4 个状态（d 的两轴，或正弦系数），在 EKF 里估计并扣除；
- 或用"自适应陷波/谐波观测器"在观测序列上估计 1.9 Hz 分量再减掉。

代价/风险：

- 需要可观测性设计，d 与目标低频机动可能混淆；
- 若目标恰好以 ω 机动，会被当成自车扰动吸收；
- 只在 S1/S2 都不可用时考虑。

### S4（缓解，不是修复）：调参/鲁棒滤波

- 先把 r_yaw/r_pitch 调到与传感器一致（0.01），否则测试没有灵敏度；但调小 R 会让杆臂误差更明显（2.3→3.6 cm @ r=5cm），这正说明问题在帧不在参。
- 低 Q 可以把 12 rad/s 平均掉，但 2–4 rad/s 慢晃更差，且目标机动滞后；不建议作为方案。
- 自适应 Q（根据 NIS 或机动检测）只能改善"目标机动 vs 平滑"的权衡，不能消除系统性杆臂误差。
- 如果一定要在现有相对系里工作，至少把 ω 纳入 Q 的自适应：高自转时降低带宽，低自转时提高带宽——但这仍然是把误差搬来搬去。

### S5（机械/战术）

- **减小水平偏置 r**：误差与 r 成正比；让云台 yaw 轴尽量靠近底盘旋转中心。z 向偏置不产生 ω×r 的水平分量（ω×r_z=0），所以主要是 x/y。
- **避开慢晃窗口**：2–4 rad/s 是最坏区（默认 R 下 11 cm），不是越快越糟；如果战术上能选择小陀螺转速，避开 EKF 带宽附近。
- **标定**：gimbal_offset 要用真实值（含相机光心），5 cm 的标定误差就是 5 cm 量级的杆臂误差。

### 方案对比

| 方案 | 需要的信息 | 瞄准误差（r=0.1, ω=12） | 改动量 | 风险 |
|---|---|---|---|---|
| S1 世界系观测 | 图像时刻 TF 平移 | ~0.15 cm | 小（solver/传感器） | TF 时间同步 |
| S2 EKF 输入 | 陀螺+里程计+杆臂 | ~0.15 cm（等价 S1） | 小（消息+3 行 predict） | v_ego 偏差积分漂移 |
| S3 谐波增广 | 仅 ω | 未实测，预计 1–3 cm | 大 | 可观测性/误吸收 |
| S4 调参 | 无 | 2–14 cm（随 Q/R 移动） | 极小 | 不能消除 |
| S5 机械 | 结构 | 与 r 成正比 | 大 | 受设计限制 |

---

## 4. 测量与可视化的修正（否则会得出错误结论）

### 4.1 common/metrics.cpp 混帧

当前：

    err_x  = est.center.x() - gt.target.center.x();   // 相对 - 绝对
    err_vx = est.velocity.x() - gt.target.vel.x();    // 相对速度 - 绝对速度
    aim_lead_bias = vel_err_norm * flight_time;       // 漏掉位置滞后项

建议至少输出 4 组：

    p_rel_err = ||est.center - (gt.target.center - gt.gimbal.pos)||; // 滤波质量（相对系）
    v_rel_err = ||est.velocity - (gt.target.vel - gt.gimbal.vel)||;  // 相对系速度质量
    v_abs_bias = ||est.velocity - gt.target.vel||;                   // 系统性帧偏差
    lead_lat  = 横向( aim_point - (gt.target.center + gt.target.vel*T) ); // 唯一与命中相关

其中 lead_lat 要投到垂直视线的平面，别用 ||aim - target|| 把距离误差也算进去。

### 4.2 NIS / NEES

tracker/extended_kalman_filter.cpp:56-66：

- 残差用的是**更新后**的 h(x) 和**后验** P；应改为先验：

      Eigen::VectorXd residual = z_subtract(z, h(x_prior));
      Eigen::MatrixXd S = H * P_prior * H.transpose() + R; // P_prior 在 update 前保存
      double nis = residual.transpose() * S.inverse() * residual;

- 阈值：dof=4，95% 上分位是 **9.488**；0.711 是 5% 下分位。当前代码会把 95% 的正常更新标成 fail。
- nees 用 (x - x_prior)'P^{-1}(x - x_prior) 是"状态更新量"，不是 NEES；仿真里应拿真值算 (x_est - x_true)'P^{-1}(x_est - x_true)。

### 4.3 渲染/3D marker 混帧

render/camera_detector_renderer.cpp：

- cam_pos = gt.gimbal.pos（世界），est.center（相对），project_world_to_cam(ekf_armor_3d, cam_pos, ...) 把相对点当世界点 → 固定偏移 ≈ p_g。
- 修法二选一：

      // A. 观测/估计统一到世界系（S1）——推荐
      // B. 若 est 是相对系，则 cam_pos 用 0，或投影前 est_world = est + gt.gimbal.pos

- （已修复）3D marker 曾混帧：aim.lead 射线从 gt.gimbal.pos（绝对）到 aim_lead_pt（相对），3D 视口的 kalman.* 与 ground_truth.* 不在同一原点。
  现在 app/main.cpp 的 publish_3d_markers 把所有 marker 统一到云台中心系（frame_id = gimbal），
  底盘/云台连杆/真值均减去 gt.gimbal.pos，aim 射线从原点射出，因此 kalman.* 与 ground_truth.* 的间距就是真实估计误差。
- main.cpp 的 compute_aim_lead_point 用相对 est 直接当世界点，同样受影响（在当前云台系口径下成立）。

---

## 5. 仿真平台建议（让测试真正有区分度）

1. **配置与 README 对齐**：当前 offset=5cm、r_yaw=0.04 的默认场景误差只有 2.3 cm，复现不出 README 的 1.2 m/s；建议加一个 "stress" 配置：offset=0.10~0.20、r_yaw=0.01、fps=100、duration=10。
2. **chassis_w 改成时间函数**：现在是常数（sim/kinematics.cpp:64），只能测单频；加 w(t)=A sin(2πft) 或 w0+Δw sin(...)，扫描 f=0.3–5 Hz。**务必包含 2–4 rad/s 慢晃**，那是最坏区。
3. **加 roll/pitch 晃动**：现在 gimbal.pitch=0、无 roll（sim/kinematics.cpp:92-95），只测了 yaw 平面。底盘悬挂的俯仰/横滚会直接改变视线，且 IMU 补偿更复杂；这是"底盘晃动"更真实的部分。
4. **加 TF/检测延迟**：tf_delay_ms 0/10/20/40；图像时刻 vs 处理时刻的 TF 混用，在 12 rad/s 下 20 ms = 0.24 rad，杆臂误差 2.4 cm，且姿态误差会被误当目标运动。
5. **加目标 z / 装甲高度 / 相机光心前置**：现在 z=0、pitch=0，垂直通道退化；真实相机光心相对云台 yaw 轴有前向偏置，会引入 ω_gimbal × r_cam。
6. **加"无扰动"和"世界系 oracle"对照**：offset=0 / ω=0 作为噪声地板；S1 作为上界；这样能分离"滤波噪声"和"系统性帧误差"。
7. **目标机动测试**：常数速度不够；加阶跃加速度/正弦机动，才能暴露 Q 的"平滑 vs 滞后"权衡，避免用低 Q 把误差藏起来。
8. **日志 CSV**：把 t, r, ω, p_rel, v_rel, v_abs, lead_lat, nis_pre, nees, id_switch 落盘，离线画频响；现在只有终端 2 s 一条的汇总，无法做频域分析。
9. **修 Target::update 的潜在误匹配**：杆臂大时，预测装甲板距离整体偏移 r，按距离排序取前 3 可能选错 id；建议记录 switch_count_/last_id，在 offset=0.2、近距（2 m）下验证。
10. **修 README 的 50 Hz/100 Hz、build.sh/run.sh 缺失**：当前 README 说 50 Hz、config 是 100 Hz，且根目录没有 build.sh/run.sh，会影响别人复现。

---

## 6. 复现

独立 benchmark（不依赖 OmniScope / OpenCV 运行库，只编译所需源文件）：

    g++ -O2 -std=c++17 experiments/bench.cpp \
        sim/kinematics.cpp sim/camera_sensor.cpp \
        tracker/target.cpp tracker/extended_kalman_filter.cpp tracker/math_tools.cpp tracker/baseline_tracker.cpp \
        -I. -Icommon -Isim -Itracker -I/usr/local/include/eigen3 -I/usr/include/opencv4 \
        -o experiments/bench
    ./experiments/bench sens      # 偏置/自转扫描
    ./experiments/bench leadfull  # 瞄准级补偿反例
    ./experiments/bench qmove     # Q 权衡
    ./experiments/bench main      # 全量表

bench 变量：

- BASE：原版（伪世界观测 + 原版瞄准）；
- FRAME：观测加 p_g（S1 上界）；
- LEAD_NAIVE：v_g*T；
- LEAD_DISP：只加云台位移；
- LEAD_FULL：加 p_g(t0) + 云台位移。

工程实测（需要 OmniScope host 可不开）：

    ./build/sim_waggle configs/sim_waggle.yaml

把 configs/sim_waggle.yaml 的 duration 设成 10、offset 设成 0.1、r_yaw 设成 0.01，看终端 Ghost Vel / Pos Err / Lead Bias 的变化；注意这些指标本身是混帧的，优先看新增的 lead_lat。

---

## 7. 一页纸行动清单

- [ ] **P0**：把 r_yaw/r_pitch 改成与 noise_yaw_std 一致（0.01），加 offset=0.1 / duration=10 的 stress 配置，先让问题可见。
- [ ] **P0**：在仿真里加 compensate_gimbal_translation 开关，跑 BASE vs S1 对照；确认 lead_lat 从 ~7 cm 降到 ~0.2 cm。
- [ ] **P1**：实现 S2（消息加 v_gimbal_world，Target::predict 减 v_ego*dt），与 S1 对齐验证。
- [ ] **P1**：修 metrics（相对/绝对分开 + 真 lead_lat）、NIS（先验残差 + 9.488）、renderer 混帧。
- [ ] **P2**：加 w(t) 正弦、roll/pitch、TF 延迟、目标 z、相机前置；扫 0.3–5 Hz。
- [ ] **P2**：把 benchmark 接进 CI，输出 CSV + 阈值（例如 lead_lat < 2 cm、v_abs_bias < 0.05 m/s）。
- [ ] **P3**：标定 gimbal_offset（含相机光心）；评估机械上减小 r 的可行性。

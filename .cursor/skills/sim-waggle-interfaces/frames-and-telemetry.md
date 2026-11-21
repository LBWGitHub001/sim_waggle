# 坐标系、曲线与 Marker

## 两套坐标

| 名字 | 原点 | 轴向 | 谁用 |
|---|---|---|---|
| **世界系** | 绑定云台中心（随云台平移） | **固定，不随云台转** | `xyz_in_world`、EKF、`kal_*`、3D Marker、`gt.target`（渲染） |
| **辅助惯性系** | 场地/底盘积分原点 | 与世界系平行 | `gt.target_inertial`、`true_*`、Kalman−真值误差 |
| 相机系 | 云台中心 | 随云台 yaw 转 | `xyz_in_gimbal` |

运动学在辅助惯性系积分，`ground_truth()` 再平移到世界系。对外 `gimbal.pos ≡ 0`、`gimbal.vel ≡ 0`，底盘绕原点公转（杆臂 = `-chassis.pos`）。`target_inertial` 保持积分系里的敌车，**不把云台公转算进敌车真值**。

观测（`sim/camera_sensor.cpp`）：

```
r_g2a         = armor_pos - gimbal.pos   // 世界系下 gimbal.pos=0
xyz_in_gimbal = R_gimbal2world^T * r_g2a
xyz_in_world  = r_g2a
```

惯性系里云台仍在公转：`p_g = p_chassis + Rz(θ_c)*d`，`|v_g|=ω|d|`。该运动变成世界系里目标的表观速度（`gt.target.vel`），**不是**敌车真值。

默认 `d=(0.05,0,0)` m，`ω_c=12` rad/s → 表观 `|v|=0.6` m/s。静止敌车时 `true_vel≈0`，`kal_vel` 会带这支鬼影。

## 谁在哪一系发数

观测 / Kalman / 3D 在世界系，禁止再减 `gimbal.pos`。`true_*` 走惯性系敌车。

| 量 | 发布点 |
|---|---|
| `obs_armor_*` | SimNode = `obs.xyz_in_world`（世界系） |
| `true_center_*` / `true_armor_*` / `true_vel_*` | SimNode = `gt.target_inertial.*` |
| `kal_center_*` / `kal_vel_*` | TrackerNode = EKF（世界系） |
| 3D Marker | `app/main.cpp`，`frame_id="world"` |
| `err_*` | `app/main.cpp` = Kalman − `target_inertial` |

`kal_center_*` 对齐 `true_center_*` 时**不应贴死**：差一个杆臂圆。`obs_armor_*` 也不再和惯性系 `true_armor_*` 重合。

## plot 键（已占用，禁止改名）

SimNode 分三次 `plot`（位置先发，看板默认勾选）：

```
obs_armor_x/y/z          m
true_center_x/y/z        m
obs_armor_yaw_deg        deg
obs_w_ego                rad/s   (= chassis.w)
true_armor_x/y/z         m
true_armor_yaw_deg       deg
true_vel_x/y/z           m/s
true_yaw_deg             deg
true_w                   rad/s
true_w_ego               rad/s
```

TrackerNode（`x.size()>=11` 才发）：

```
kal_center_x/y/z         m
kal_vel_x/y/z            m/s
kal_yaw_deg              deg
kal_w                    rad/s
kal_radius_mm            mm
```

main（Kalman − 真值）：

```
err_x/y/z                m
err_vx/y/z               m/s
err_yaw_deg              deg
err_w                    rad/s
err_r_mm                 mm
err_pos / err_vel        m / m/s
```

分支上可另 `plot` 新键。禁止改名已有键。

## 3D Marker 命名空间

`app/main.cpp::publish_3d_markers`，`omniscope::RemoteLogger::instance().plot_markers`。

3D 视口只画 Kalman 状态，不画真值。每帧对 `ground_truth.target` / `ground_truth.armor` 发 `DELETEALL`，清掉 Host 里按 ns:id 常驻的旧真值。

| ns | id | 类型 | 含义 |
|---|---|---|---|
| `my_robot.chassis` | 0 / 1 | SPHERE / ARROW | 底盘 + 航向 |
| `my_robot.gimbal` | 10 / 11 / 12 | LINE_LIST / SPHERE / ARROW | 杆臂、云台原点、瞄准 |
| `kalman.center` | 50 | SPHERE | 估计中心 |
| `kalman.vel` | 52 | ARROW | 估计速度（`|v|>0.05` 才画） |
| `kalman.armor` | 60+i / 70 | CUBE / LINE_LIST | 估计板 + 轮辐 |
| `aim.lead` | 80 / 82 | SPHERE / LINE_LIST | 超前落点；射线从世界原点（云台）射出 |

装甲几何常量：灯条 56 mm、小板宽 135 mm、厚 15 mm。姿态 `armor_quat(yaw)` 带 15° pitch。

## 图像

`plot_image(frame, {{"name", "camera_detector"}})`，800×600。

## 日志

`logger.log("INFO", fmt, ...)`。启动行含 tracker 名、`chassis_w`、offset、fps；每 2 s 打 abs/rel 误差、RMS(rel)、aim_bias、NIS、`obs_channel.size()`。

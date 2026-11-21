# 坐标系、曲线与 Marker

## 三系

| 名字 | 原点 | 轴向 | 谁用 |
|---|---|---|---|
| **odom** | 场地/底盘积分原点 | 固定 | 运动学、TF 中间件。滤波 / plot / 误差 **不用** |
| **gimbal** | 云台中心 | 随云台 yaw | 观测、EKF、`obs_*`/`true_*`/`kal_*`/`err_*`、Kalman Marker |
| **robot** | 敌车中心 | 随敌车 yaw | 装甲板绑定；`robot.armor` Marker |

运动学在 odom 积分：`gimbal.pos = chassis.pos + Rz(θ_c)*d`，装甲 `p_odom = robot.center + Rz(robot.yaw)*p_robot`。`ground_truth()` 返回 odom。日常使用 `express_target_in_gimbal(gt)`（`common/frames.hpp`）。

观测（`sim/camera_sensor.cpp`）：

```
r_g2a         = armor_odom - gimbal.pos
xyz_in_gimbal = R_gimbal2odom^T * r_g2a   // 工作系；加噪、ypd、yaw 都在这
xyz_in_world  = armor_odom                // 中间件，滤波和 plot 不用
yaw_raw       = armor_yaw_odom - gimbal.yaw
```

`v_gimbal = R^T * (v_odom - gimbal.vel)`，不做 ω×r。默认 `d=0.05` m、`ω_c=12` rad/s 时云台公转 `|v_g|≈0.6` m/s，会出现在 **gimbal 系真值速度**里，Kalman 会去跟。

要看场地上敌车几乎静止：Host 3D 查看系切到 `odom`。

## 谁发什么

`omniscope::plot` / `plot_markers` / `plot_tfs` **禁止写在 main**。

| 量 | 发布点 | 系 |
|---|---|---|
| `obs_armor_*` | SimNode = `obs.xyz_in_gimbal` | gimbal |
| `true_center_*` / `true_armor_*` / `true_vel_*` | SimNode = `express_target_in_gimbal(gt)` | gimbal |
| `kal_center_*` / `kal_vel_*` | TrackerNode = EKF | gimbal |
| `err_*` | TrackerNode = Kalman − gimbal 真值 | gimbal |
| TF | SimNode `plot_tfs`：`odom→gimbal`，`odom→robot` | odom 中间件 |
| `my_robot.*` / `robot.armor` | SimNode Marker | chassis / gimbal / robot |
| `kalman.*` / `aim.lead` | TrackerNode Marker，`frame_id=gimbal` | gimbal |
| 相机 HUD | main `plot_image` 读 `TrackerOutput.metrics` | 渲染 cam_pos=0 |

`kal_*` 对齐 `true_*` 时应接近（噪声 + 装甲跳跃），不是杆臂圆。杆臂表现在 gimbal 系的表观速度上。

## plot 键（已占用，禁止改名）

SimNode 分三次 `plot`（位置先发，看板默认勾选）：

```
obs_armor_x/y/z          m     gimbal
true_center_x/y/z        m     gimbal
obs_armor_yaw_deg        deg
obs_w_ego                rad/s   (= chassis.w)
true_armor_x/y/z         m     gimbal
true_armor_yaw_deg       deg
true_vel_x/y/z           m/s   gimbal
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
err_x/y/z                m
err_vx/y/z               m/s
err_yaw_deg              deg
err_w                    rad/s
err_r_mm                 mm
err_pos / err_vel        m / m/s
```

分支上可另 `plot` 新键。禁止改名已有键。禁止在 main 里 `plot`。

## 3D Marker 命名空间

| ns | id | 类型 | 发布 | frame_id |
|---|---|---|---|---|
| `my_robot.chassis` | 0 / 1 | SPHERE / ARROW | SimNode | `odom` |
| `my_robot.gimbal` | 11 / 12 | SPHERE / ARROW | SimNode | `gimbal` |
| `robot.armor` | i | CUBE | SimNode | `robot`（板位固定） |
| `kalman.center` | 50 | SPHERE | TrackerNode | `gimbal` |
| `kalman.vel` | 52 | ARROW | TrackerNode | `gimbal` |
| `kalman.armor` | 60+i / 70 | CUBE / LINE_LIST | TrackerNode | `gimbal` |
| `aim.lead` | 80 / 82 | SPHERE / LINE_LIST | TrackerNode | `gimbal`（射线从原点） |

装甲几何常量：灯条 56 mm、小板宽 135 mm、厚 15 mm。姿态 `armor_quat(yaw)` 带 15° pitch。

Host 3D 侧栏按 `ns` 分组。当前查看系只列出 TF 里真实存在的系（`odom` / `gimbal` / `robot`），**不要** `world`、`chassis`。选中某系时**只显示 `frame_id` 等于该系的 Marker**（不是把所有 Marker 变到该系原点）。

TF 树：

```
odom → gimbal
odom → robot
```

## 图像

`plot_image(frame, {{"name", "camera_detector"}})`，800×600。main 渲染：`cam_pos=0`，敌车已变到 gimbal。

## 日志

`logger.log("INFO", fmt, ...)`。启动行含 tracker 名、`chassis_w`、offset、fps；每 2 s 打 Kalman−gimbal 真值、RMS、aim_bias、NIS、`obs_channel.size()`。

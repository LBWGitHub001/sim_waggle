# 坐标系、曲线与 Marker

## 四系

| 名字 | 原点 | 轴向 | 谁用 |
|---|---|---|---|
| **odom** | 场地/底盘积分原点 | 固定 | 运动学、TF 中间件。滤波 / plot / 误差 **不用** |
| **gimbal** | 云台中心 | **与 odom 平行、不转** | 观测、EKF、`obs_*`/`true_*`/`kal_*`/`err_*`、Kalman Marker |
| **camera** | 同一云台中心 | 随 look-at yaw | HUD、光轴 Marker、TF |
| **robot** | 敌车中心 | 随敌车 yaw | 装甲板绑定；`robot.armor` Marker |

运动学在 odom 积分：`gimbal.pos = chassis.pos + Rz(θ_c)*d`，`camera.pos = gimbal.pos`，`camera.yaw = atan2(target - gimbal.pos)`。装甲 `p_odom = robot.center + Rz(robot.yaw)*p_robot`。四装甲：对侧 (id 0,2) 半径 `r`、相对车心 `z=0`；邻侧 (id 1,3) 半径 `r+l`、`z=h`（与 EKF `x[8,9,10]` 一致）。`ground_truth()` 返回 odom。日常滤波用 `express_target_in_gimbal(gt)`；HUD 用 `express_target_in_camera(gt)`（`common/frames.hpp`）。

观测（`sim/camera_sensor.cpp`）：

```
xyz_in_gimbal = armor_odom - gimbal.pos   // 工作系；加噪、ypd、yaw 都在这
xyz_in_world  = armor_odom                // 中间件，滤波和 plot 不用
yaw_raw       = armor_yaw_odom            // gimbal yaw ≡ 0
```

`v_gimbal = v_odom - gimbal.vel`（不转，正好是位置导数）。默认 `d=0.05` m、`ω_c=12` rad/s 时 `true_center_y ≈ -0.05 sin(θ_c)`，`|v_g|≈0.6` m/s，Kalman 会去跟。

要看场地上敌车几乎静止：Host 3D 查看系切到 `odom`。切 `camera` 看光轴锁定（敌车在 +X）。

## 谁发什么

`omniscope::plot` / `plot_markers` / `plot_tfs` **禁止写在 main**。

| 量 | 发布点 | 系 |
|---|---|---|
| `obs_armor_*` | SimNode = `obs.xyz_in_gimbal` | gimbal |
| `true_center_*` / `true_armor_*` / `true_vel_*` | SimNode = `express_target_in_gimbal(gt)` | gimbal |
| `kal_center_*` / `kal_vel_*` | TrackerNode = EKF | gimbal |
| `err_*` | TrackerNode = Kalman − gimbal 真值 | gimbal |
| TF | SimNode：`odom→gimbal`（平移，R=I），`gimbal→camera`（Rz look-at），`odom→robot` | |
| `my_robot.*` / `robot.armor` | SimNode Marker | odom / gimbal / camera / robot |
| `kalman.*` / `aim.lead` | TrackerNode Marker，`frame_id=gimbal` | gimbal |
| 相机 HUD | main `plot_image` 读 `TrackerOutput.metrics` | camera：`express_target_in_camera` |

`kal_*` 对齐 `true_*` 时应接近（噪声 + 装甲跳跃）。杆臂公转出现在 gimbal 的 `true_center_y` / `true_vel_*`。

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
kal_l_mm / kal_h_mm      mm
true_l_mm / true_h_mm    mm
err_x/y/z                m
err_vx/y/z               m/s
err_yaw_deg              deg
err_w                    rad/s
err_r_mm / err_l_mm / err_h_mm  mm
err_pos / err_vel        m / m/s
cmd_target_yaw/pitch_deg deg   无约束射击角
cmd_yaw/pitch_deg        deg   云台目标（TinyMPC 可行角）
cmd_yaw_vel/acc          rad/s, rad/s²
cmd_pitch_vel/acc
cmd_fly_time             s
true_camera_yaw_deg      deg   GT look-at
true_aim_yaw_deg         deg   可见装甲 atan2
```

分支上可另 `plot` 新键。禁止改名已有键。禁止在 main 里 `plot`。

## 3D Marker 命名空间

| ns | id | 类型 | 发布 | frame_id | 颜色 |
|---|---|---|---|---|---|
| `my_robot.chassis` | 0 / 1 | SPHERE / ARROW | SimNode | `odom` | 青 |
| `my_robot.gimbal` | 11 / 12 | SPHERE / ARROW | SimNode | `gimbal` | 橙（轴向=odom） |
| `my_robot.camera` | 21 / 22 | SPHERE / ARROW | SimNode | `camera` | 浅蓝（光轴） |
| `robot.center` | 0 | SPHERE | SimNode | `robot`（原点） | **全绿（真值）** |
| `robot.armor` | i | CUBE | SimNode | `robot`（板位固定） | **全绿（真值）** |
| `kalman.center` | 50 | SPHERE | TrackerNode | `gimbal` | **黄（Kalman 中心）** |
| `kalman.vel` | 52 | ARROW | TrackerNode | `gimbal` | 黄（随中心） |
| `kalman.armor` | 60+i / 70 | CUBE / LINE_LIST | TrackerNode | `gimbal` | **蓝（Kalman 装甲）** |
| `aim.lead` | 80 / 82 | SPHERE / LINE_LIST | TrackerNode | `gimbal`（射线从原点） | 品红 |

装甲几何常量：灯条 56 mm、小板宽 135 mm、厚 15 mm。姿态 `armor_quat(yaw)` 带 15° pitch。

Host 3D 侧栏按 `ns` 分组。当前查看系只列出 TF 里真实存在的系（`odom` / `gimbal` / `camera` / `robot`），**不要** `world`、`chassis`。选中某系时，**全部 Marker 经 TF 变到该系再画**：

- `gimbal`：云台在原点、轴向固定；敌车自转 + 杆臂造成的中心 y 摆动
- `camera`：光轴在 +X，敌车在前方
- `robot`：敌车原点在显示原点，装甲板相对固定
- `odom`：全场。敌车自转 + 中心平动，云台随底盘转

不要再按 `frame_id` 过滤掉其他系的 Marker。场景原点不要再画那组巨大的三箭头；各系姿态用「显示 TF 坐标轴」的小三轴。

TF 树：

```
odom → gimbal → camera
odom → robot
```

## 图像

`plot_image(frame, {{"name", "camera_detector"}})`，800×600。main 渲染：`cam_pos=0`，敌车 `express_target_in_camera`；Kalman 从 gimbal 旋到 camera 再画。

## 日志

`logger.log("INFO", fmt, ...)`。启动行含 tracker 名、`chassis_w`、offset、fps；每 2 s 打 Kalman−gimbal 真值、RMS、aim_bias、NIS、`obs_channel.size()`。

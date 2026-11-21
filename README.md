# sim_waggle: 云台-底盘偏置公转对自瞄 Kalman 影响仿真平台

本工程是一个独立的 C++ 仿真测试平台。从 RoboMaster 自瞄代码库提取原版 Kalman，模拟**云台旋转中心与底盘旋转中心不重合**（偏置 $d$）在底盘小陀螺 / 摆动（$\omega_c$）时对自瞄后端滤波的影响。

当前分支是 **`exp/ego-acc`**：在方向不变的 `gimbal` 工作系里，把自车传感器得到的 xy 加速度当作 EKF 控制量 $u$ 注入过程模型。这是相对 `main` 上 `BaselineTracker` 的核心突破，完整推导见第 3 节。

OmniScope 作只读 SDK：`omniscope::plot` / `plot_markers` / `plot_tfs` 只写在 **SimNode** 与 **TrackerNode**；`main` 只允许 `plot_image`。

---

## 1. 工程架构（多线程节点 + 物理隔离独立包）

`sim/` 与 `tracker/` 是两个独立 CMake 包，编译期互相看不见头文件，只通过 `common/` 消息耦合。

**运行时**：`SimNode` 以 YAML 的 `fps`（默认 100 Hz）推进物理，观测进 FIFO `Channel`；`TrackerNode` 事件驱动消费——首帧只 `update`，之后 `predict` → `control` → `update`，不丢帧。误差在 TrackerNode（Kalman − gimbal 系真值）。main 只读 `LatestSlot` 做 HUD。

四系：`odom`（惯性中间件）→ `gimbal`（原点绑云台、轴向与 odom 平行**不转**，日常滤波 / plot / 误差都在此）→ `camera`（同原点、look-at yaw，只给 HUD/TF）；`robot` 绑装甲板。

```
sim_waggle/
├── configs/sim_waggle.yaml
├── common/                     # 两节点唯一耦合
├── sim/                        # 运动学、相机观测、SimNode
├── tracker/
│   ├── ego_control_tracker.*   # 本分支：自车加速度控制注入
│   ├── baseline_tracker.*      # bench 对照，不调 control
│   ├── target.* / extended_kalman_filter.*
│   └── tracker_node.*
├── render/                     # 车载 HUD（camera 系）
└── app/main.cpp                # YAML + 接线 + plot_image
```

---

## 2. 快速上手

依赖：GCC 11+ / C++17，CMake 3.16+，Eigen3，OpenCV，fmt，yaml-cpp，nlohmann_json，Python 3.9+（Host）。

```bash
cmake -B cmake-build-debug -DCMAKE_BUILD_TYPE=Debug
cmake --build cmake-build-debug
```

```bash
# 门户 http://127.0.0.1:8080
./thirdparty/OmniScope/host/start.sh
./cmake-build-debug/sim_waggle configs/sim_waggle.yaml
```

调参编辑 `configs/sim_waggle.yaml`（`chassis_w`、`gimbal_offset`、`measurement.noise_acc_std`、`ekf.M`）。

---

## 3. 本分支突破：自车加速度作为 EKF 控制量

这一节是 `exp/ego-acc` 的算法正文。读完应能不看源码复现模型、噪声约定和每帧调用序。

### 3.1 问题：基线 CV 把视在加速度当未知扰动

工作系是 **方向不变的 `gimbal`**：原点跟随云台中心，轴向始终与场地惯性系 `odom` 平行。敌车在 odom 里几乎静止时，gimbal 里看到的中心轨迹是云台公转造成的**视在运动**，不是敌车在场地上跑。

`main` 的 `BaselineTracker` 用常速度（CV）过程模型，把 xy 上的视在加速度塞进过程噪声 $Q$（`q_xyz_acc_var`）。结果是零和：

- $Q$ 太小：跟不上杆臂公转，位置滞后；
- $Q$ 太大：把视在速度当成敌车速度，瞄准点被伪速度带走。

调 $Q/R$ 消不掉几何项。瞄准点上事后加 $v_g T$ 更差（相位滞后会补过头）。把滤波搬到 `odom` 是另一条实验，不是本分支。

### 3.2 相对运动学：已知的是 $-a_g$

记敌车中心在 odom 为 $\mathbf{p}_t$，云台原点为 $\mathbf{p}_g$。gimbal 不转，所以相对位置就是平移差：

\[
\mathbf{p}_{\mathrm{rel}} = \mathbf{p}_t - \mathbf{p}_g
\]

对时间求两次导：

\[
\ddot{\mathbf{p}}_{\mathrm{rel,xy}} = \mathbf{a}_{t,\mathrm{xy}} - \mathbf{a}_{g,\mathrm{xy}}
\]

底盘小陀螺时 $|\mathbf{a}_g|$ 由杆臂 $d$ 与 $\omega_c$ 决定，量级远大于敌车平移加速度。未知机动留给过程噪声；**已知的主导项是 $-\mathbf{a}_g$**。把它当作控制量，滤波器就不必再估计「云台自己在转」这件事。

仿真里 $\mathbf{a}_g$ 来自云台速度的有限差分（`KinematicsEngine::origin_acc()`）。发给 tracker 的理想控制是

\[
\hat{\mathbf{u}} = (-a_{g,x},\, -a_{g,y})^\top
\]

（z 不加控制：竖直方向没有这套杆臂几何。）

### 3.3 离散模型

11 维状态与基线相同：

\[
\mathbf{x} = [x_c,\, v_x,\, y_c,\, v_y,\, z_c,\, v_z,\, \psi,\, \omega,\, r,\, l,\, h]^\top
\]

xy 用已知加速度的常加速离散，其余仍 CV。真实过程是

\[
\mathbf{x}_k = F \mathbf{x}_{k-1} + B(\mathbf{u}_k + \mathbf{m}_k) + \mathbf{w}_k
\]

其中：

- $F$：各通道 CV 块 $\begin{bmatrix}1 & dt\\ 0 & 1\end{bmatrix}$（位置-速度）以及 $r,l,h$ 的恒等。
- $\mathbf{u}=(u_x,u_y)^\top$：tracker **实际吃到**的加速度。仿真侧 $\mathbf{u}=\hat{\mathbf{u}}+\mathbf{m}$，不是完美 IMU。
- $\mathbf{m}\sim\mathcal{N}(\mathbf{0},M)$。仿真侧 $\sigma_u$ 是 YAML `measurement.noise_acc_std`（各轴独立 $\mathcal{N}(0,\sigma_u^2)$）；滤波侧 $M$ 是 YAML `ekf.M`（2×2，默认 $I_2$）。对角元应对齐 $\sigma_u^2$。
- $\mathbf{w}\sim\mathcal{N}(\mathbf{0},Q)$：只覆盖**未建模**的 $z$、$\psi$（分段白噪声，参数仍是 `q_xyz_acc_var` / `q_yaw_acc_var`）以及敌车真实机动残差。**xy 不再走原来的未知加速度 $Q_{xyz}$ 块**——未知量已经在 $u$ 上。
- 假设 $\mathbf{w}\perp\mathbf{m}$，没有交叉项 $S=\mathbb{E}[\mathbf{w}\mathbf{m}^\top]$。

控制矩阵 $B\in\mathbb{R}^{11\times 2}$ 只把 xy 加速度映射进 $[x_c,v_x,y_c,v_y]$：

\[
B =
\begin{bmatrix}
\tfrac12 dt^2 & 0 \\
dt & 0 \\
0 & \tfrac12 dt^2 \\
0 & dt \\
\mathbf{0}_{7\times 2}
\end{bmatrix}
\]

观测方程与基线相同：YPDA $\mathbf{z}=[\mathrm{yaw},\,\mathrm{pitch},\,\mathrm{dist},\,\mathrm{armor\_yaw}]^\top$，雅可比仍在 `Target::h_jacobian`。后验协方差用约瑟夫形式

\[
P = (I-KH)P(I-KH)^\top + KRK^\top
\]

（比 $(I-KH)P$ 数值更稳）。

### 3.4 为什么无噪声 $u$ 会「零误差」

若仿真把真值 $\hat{\mathbf{u}}=-a_g$ **原样**交给 tracker，且滤波里 $M=0$，则过程模型与 gimbal 真值运动学是同一条轨迹。观测只剩相机噪声，中心误差会被压到噪声地板。这不是滤波神奇，是模型吃到了完美输入。

因此两件事必须同时成立：

1. **仿真**给 `ego_acc` 加 $m$（`enable_noise` 打开时 xy 各加 $\mathcal{N}(0,\sigma_u^2)$，RNG 种子 43，与相机观测种子 42 独立，避免人为相关）。
2. **滤波** `control` 步必须加 $BMB^\top$，让 $P$ 承认 IMU 不准。

曲线：`u_true_x/y` 是无噪声 \(\hat{\mathbf{u}}\)；`u_x/y`（与 `ego_acc_x/y` 同值）是 tracker 实际吃到的 \(\mathbf{u}=\hat{\mathbf{u}}+\mathbf{m}\)。

### 3.5 三步 API（对应代码）

把 $F$、$B$、$Q$ 拆开，避免把过程噪声混进 $F$ 那一步。每帧（已初始化）严格按序：

1. **predict** `ekf.predict(F)`（Target 里用 $f$ 对 yaw 做 `limit_rad`）

   \[
   \hat{x}\leftarrow F\hat{x},\qquad P\leftarrow FPF^\top
   \]

2. **control** `ekf.control(B,u,M)`

   \[
   \hat{x}\leftarrow \hat{x}+Bu,\qquad P\leftarrow P+BMB^\top
   \]

3. **update** `ekf.update(z,H,R,Q,\ldots)`：先把未建模过程噪声加进先验，再量测更新

   \[
   P\leftarrow P+Q
   \]

   然后算 $K$、校正 $z$。三步合起来，量测前的先验协方差就是

   \[
   P^- = FPF^\top + BMB^\top + Q
   \]

`TrackerNode` 调用序：

```
首帧:  update(obs)
之后:  predict(dt) → control(msg.ego_acc) → update(obs)
```

`EgoControlTracker::predict` 只建 $F$；`control` 只建 $B,u,M$；`Target::update_ypda` 用上一拍 `last_dt_` 组 $Q_{z,\psi}$（xy 块为 0）。首帧尚未 predict 时 `last_dt_=0`、$Q=0$。

本分支 `BaselineTracker`（bench）仍只 `predict(dt)`，不调 `control`。

### 3.6 和基线 / 其它方案的对比

| 做法 | 过程模型对 xy 的态度 | 本分支是否采用 |
|---|---|---|
| `main` `BaselineTracker` | 未知加速度进 $Q_{xyz}$，无 $B,u$ | 否（对照） |
| 本分支 `EgoControlTracker` | 已知 $-a_g$ 进 $Bu$，未知量是 IMU 噪声 $m$ 与 $z/\psi$ 的 $w$ | 是 |
| 把滤波搬到 `odom` | 原点不再绑云台，视在运动消失 | 否，日常状态/误差/plot 仍在不转的 gimbal |
| 瞄准点事后加 $v_g T$ | 不改正过程模型 | 否，已证实会更差 |

### 3.7 YAML

| 字段 | 含义 | 默认 |
|---|---|---|
| `measurement.enable_noise` | 同时开关相机噪声与加速度噪声 | true |
| `measurement.noise_acc_std` | 仿真 IMU 高斯噪声标准差 $\sigma_u$（m/s²） | 1.0 |
| `ekf.M` | 控制噪声协方差 $2\times 2$（(m/s²)²），`P += B M Bᵀ` | `[[1,0],[0,1]]` |
| `ekf.q_xyz_acc_var` | 本分支只还用于 **$z$** 的分段白噪声 | 100 |
| `ekf.q_yaw_acc_var` | $\psi$ 角加速度方差 | 75 |

`M` 对角元应对齐 `(noise_acc_std)²`。只改一边会让 NIS / 误差和模型不一致。仍接受旧字段 `ekf.q_u_acc_var`（标量，当作 $M=\sigma^2 I_2$）。

### 3.8 怎样判断实验成立

- `u_x/y` 应在 `u_true_x/y` 附近抖动，不是光滑曲线。
- `err_x/y`、`err_vx/vy` 相对 `main` 基线应明显下降，但**不应再贴成数值零**（有 $m$ 和 $M$）。
- Host 3D 切到 `odom`：场地上敌车几乎静止；切到 `gimbal`：杆臂视在运动应由控制量吃掉，而不是整段被 $Q$ 当未知扰动。

---

## 4. 3D Marker 与看板

浏览器打开 **http://127.0.0.1:8080** 的 Watch 页（2×2）：

1. **右上 3D**：Host 把所有 Marker 变到当前查看系（RViz 风格，不是按 `frame_id` 过滤）。真值全绿（`robot`）；Kalman 装甲蓝、中心黄（`frame_id=gimbal`）。切 `odom` 看场地，切 `camera` 看光轴锁定。
2. **左下相机**：`camera_detector` HUD，在 look-at 的 `camera` 系。
3. **左上波形**：工作系 = 不转的 `gimbal`。
   - SimNode：`obs_armor_*`、`true_center_*` / `true_vel_*` / `true_armor_*`、`u_true_x/y`、`u_x/y`（及同值的 `ego_acc_x/y`）
   - TrackerNode：`kal_*` 以及误差 `err_*`（Kalman − gimbal 真值；**不在 main 里 plot**）
4. **右下日志**。

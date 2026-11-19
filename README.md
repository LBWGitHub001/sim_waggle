# sim_waggle: 云台-底盘偏置公转对自瞄 Kalman 影响仿真平台

本工程是一个**独立、完整、现代高内聚架构**的 C++ 仿真测试平台。从 RoboMaster 自瞄代码库 `auto_aim_` 提取了原版 Kalman 滤波逻辑，模拟**云台旋转中心与底盘旋转中心不重合（偏置 $R_d$）在底盘小陀螺/摆动（$\omega_c$）时对自瞄后端 Kalman 滤波产生的影响**。

本工程严格贯彻**对 OmniScope 零侵入原则**（Zero-Intrusion），仅将 [OmniScope](https://github.com/LBWGitHub001/OmniScope.git) 视作只读外部 SDK，通过独立的 `bridge/` 隔离层交互，提供**标准 ROS 2 3D Marker 交互场景**、实时遥测数据波形与高逼真度**车载相机目标检测图像 (`camera_detector`)**。

---

## 1. 工程架构（多线程节点 + 物理隔离独立包）

仿真与追踪器拆分为**两个类 ROS Node 的独立线程**，通过线程安全的消息通道通信；
代码上 `sim/` 与 `tracker/` 是**两个独立 CMake 子工程**，include 路径按包隔离——
编译 sim 包时看不到 tracker 头文件（反之亦然），两包仅通过 `common/` 的消息类型耦合。

**运行时数据流**：`SimNode` 线程以 50Hz 推进物理仿真，发布观测到 FIFO `Channel`；
`TrackerNode` 线程事件驱动逐条消费（首帧 init，之后每帧 predict+update，**不丢帧、数值可复现**），
估计快照经 `LatestSlot` 发布；main 线程只读最新快照做可视化（metrics / bridge / render）。

```
sim_waggle/
├── CMakeLists.txt              # 顶层聚合：add_subdirectory 三个独立包 + 可执行目标
├── build.sh                    # 一键构建脚本
├── run.sh                      # 一键运行脚本
├── configs/
│   └── sim_waggle.yaml         # 仿真、相机、滤波与遥测配置文件
├── thirdparty/
│   └── OmniScope/              # 外部只读第三方库 (100% 纯净，零侵入修改)
│
├── common/                     # 【包 omni_common】共享消息包（等价 ROS msgs，两节点唯一耦合点）
│   ├── CMakeLists.txt
│   ├── types.hpp               # 领域模型 + StampedObservation / TrackerOutput 话题消息
│   ├── channel.hpp             # Channel(FIFO 队列) 与 LatestSlot(最新值槽) 线程安全原语
│   ├── metrics.hpp             # 误差评估器 + compute_aim_lead_point（快照侧瞄准点计算）
│   └── metrics.cpp
│
├── sim/                        # 【包 sim_node】仿真节点（编译期看不到 tracker 头文件）
│   ├── CMakeLists.txt
│   ├── kinematics.hpp/.cpp     # 运动学引擎：积分量(chassis/target)与派生量(gimbal/armor/可见性)分离
│   ├── camera_sensor.hpp/.cpp  # 可见装甲板观测生成器（伪世界系坐标 + 平面高斯噪声）
│   └── sim_node.hpp/.cpp       # SimNode 线程：50Hz 发布观测 FIFO + 真值快照，确定性仿真时钟
│
├── tracker/                    # 【包 tracker_node】追踪节点（编译期看不到 sim 头文件）
│   ├── CMakeLists.txt
│   ├── itracker.hpp            # 抽象追踪器接口 (为 Phase 2 偏置补偿算法提供多态基类)
│   ├── baseline_tracker.hpp/.cpp  # 原版未补偿 EKF 基线实现
│   ├── target.hpp/.cpp         # 目标运动模型与 11 维状态转移/观测雅可比
│   ├── extended_kalman_filter.hpp/.cpp # 约瑟夫形式协方差更新与卡方检验
│   ├── armor.hpp               # 装甲板几何定义
│   ├── math_tools.hpp/.cpp     # 角度归一化与坐标转换
│   └── tracker_node.hpp/.cpp   # TrackerNode 线程：事件驱动消费观测、发布估计快照
│
├── render/                     # 车载相机检测图像渲染器（app 侧私有）
│   ├── camera_detector_renderer.hpp # 800x600 针孔透视、发光 LED 灯条、YOLO 框与 HUD
│   └── camera_detector_renderer.cpp
│
├── bridge/                     # OmniScope 隔离适配器（app 侧私有, Adapter Pattern）
│   ├── omniscope_bridge.hpp    # 封装 RemoteLogger，下发 ROS 2 3D Marker 与遥测波形
│   └── omniscope_bridge.cpp
│
├── app/                        # 接线层
│   └── main.cpp                # load_config() 一处解析 YAML；创建话题通道、启动两节点线程；main 循环只做可视化
│
└── scripts/
    └── run_host.sh             # 启动 OmniScope Host 门户脚本
```

---

## 2. 快速上手

### 2.1 依赖环境
- GCC 11+ / Clang (支持 C++17)
- CMake 3.16+
- Eigen3, OpenCV, fmt, yaml-cpp, nlohmann_json
- Python 3.9+ (用于运行 OmniScope Host Web 界面)

### 2.2 一键编译
```bash
./build.sh
```

### 2.3 运行仿真与可视化
```bash
# 步骤 1：启动 OmniScope Web 调试门户 (http://127.0.0.1:8080)
./scripts/run_host.sh

# 步骤 2：启动仿真程序
./run.sh
```

调参请直接编辑 `configs/sim_waggle.yaml`（底盘转速 `chassis_w`、偏置 `gimbal_offset`、时长 `duration` 等）；`run.sh` 仅接受一个可选参数指定其他 YAML 配置路径：
```bash
./run.sh [path/to/config.yaml]
```

---

## 3. 3D Marker 与看板交互体系

在浏览器访问 **`http://127.0.0.1:8080`** 进入 **Watch** 页面，将呈现 **2x2 工业看板**：
1. **右上 3D 视口**：全交互式 **Three.js 3D 视口**，实时渲染包含 8 大命名空间的标准化 ROS 2 Markers：
   - `my_robot.chassis`（青色底盘球体 + 航向箭头指针 + 角速度标签）
   - `my_robot.gimbal`（黄色偏置杆臂 + 公转云台中心球体 + 瞄准朝向箭头）
   - `ground_truth.target` & `ground_truth.armor`（真值目标中心、4 块空间装甲板长方体及轮辐）
   - `kalman.center` & `kalman.armor`（EKF 估计中心与拟合装甲板）
   - `kalman.vel`（**警示红橙色 3D 箭头**，随底盘旋转以 $\approx 1.2\,\text{m/s}$ 疯狂公转变向，直观暴露正弦伪速度）
   - `aim.lead`（金黄色弹道超前瞄准落点球体与火控射线）
2. **左下相机视口**：实时的 **`camera_detector`** 车载相机检测图像（荧光蓝 LED 灯条、端点角点、绿框及随伪速度剧烈绕圈偏摆的黄色 `AIM LEAD` 准星）；
3. **左上波形视口**：**估计误差曲线**（`bridge::OmniScopeBridge::publish_telemetry` 发布，单位见键名）：
   - 默认勾选的 6 条核心曲线（统一折算为脱靶量 mm，量级可比）：
     `pos_err_mm` / `rel_pos_err_mm`（位置误差）、`lead_bias_mm` / `rel_lead_bias_mm`（速度误差×弹道飞行时间的落点脱靶量）、`err_pos_x_mm` / `err_pos_y_mm`（分轴位置误差）
   - 侧栏可勾选的明细通道：`vel_err_mm_s`、`rel_vel_err_mm_s`、`err_vel_*_mm_s`、`rel_err_*_mm`、`err_yaw_deg`、`err_w_deg_s`、`armor_err_mm`、`nis`、`rmse_*_mm`，以及 `obs_*` / `target_*` / `true_v*` 原始量
   - 两套误差口径：`pos_err_mm` / `lead_bias_mm` 为**绝对误差**（估计 vs 世界系真值，含云台公转平移）；
     `rel_pos_err_mm` / `rel_lead_bias_mm` 为**相对误差**（估计 vs 云台系相对真值，即观测所在坐标系，反映 EKF 自身精度）
   - 另有 XY 曲线 `err_locus`：云台系 x-y 平面误差轨迹（面板类型切到「XY 曲线」即可查看）
4. **右下日志视口**：实时结构化日志控制台。

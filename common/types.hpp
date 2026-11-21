#ifndef COMMON__TYPES_HPP
#define COMMON__TYPES_HPP

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace common
{

// =============================================================================
// 坐标系
//   odom   场地惯性系（中间件）。运动学在此积分；仅 TF 使用这些坐标。
//   gimbal 云台中心、轴向与 odom 平行（不转）。观测、EKF、plot、误差都在此。
//   camera 与 gimbal 同原点，轴向随 look-at yaw。只给 HUD / 光轴 TF。
//   robot  敌车体。装甲板绑在此系；robot 本身在 odom 中转动。
//
// GroundTruth 的 chassis / gimbal.pos / camera / target 位姿都是 odom。
// 日常使用请走 frames.hpp 的 express_*_in_gimbal()，不要直接拿 odom 坐标滤波。
// xyz_in_gimbal / EKF / kal_* / err_* 在方向不变的 gimbal。
// xyz_in_world 保留为装甲在 odom 的位置，滤波和 plot 不用。
// =============================================================================

// 1. 本车底盘状态（odom）
struct ChassisState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0; // 自转角速度 (rad/s)
};

// 2. 本车云台工作系原点（odom）。轴向与 odom 平行：yaw=0，R=I
struct GimbalState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
};

// 2b. 相机/光轴（与 gimbal 同原点）。R_camera2odom = Rz(look-at yaw)
struct CameraState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Matrix3d R_camera2odom = Eigen::Matrix3d::Identity();
};

// 3. 敌车目标真实状态（odom：center/yaw/armors_pos；装甲几何绑在 robot）
struct TargetState
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0; // 自转角速度 (rad/s)
  double radius = 0.20;
  int armor_num = 4;
  std::vector<Eigen::Vector3d> armors_pos;
  std::vector<double> armors_yaw;
};

// 4. 仿真环境综合真实物理状态（odom 位姿，给 TF / 中间件）
struct GroundTruth
{
  double t = 0.0;
  ChassisState chassis;
  GimbalState gimbal;
  CameraState camera;
  TargetState target;

  // 当前由视觉传感器认定的最佳可见装甲板 id，
  // 位姿请按 id 索引 target.armors_pos / target.armors_yaw（odom）
  int visible_armor_id = 0;
};

// 5. 传感器产生的装甲板视觉测量 (Observation)
struct ArmorObservation
{
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero(); // 工作系（原点=云台，轴向=odom）
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();  // 装甲在 odom（中间件，滤波不用）
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero(); // yaw, pitch, distance（由 xyz_in_gimbal）
  double yaw_raw = 0.0; // 装甲 yaw 在方向不变的 gimbal（= odom yaw）
  double confidence = 0.98;
  std::string name = "one";
  std::string color = "blue";
  std::string type = "small";
  double width = 0.135;
  double height = 0.056;
};

// 6. 算法给出的目标状态估计（gimbal）
struct TargetEstimate
{
  Eigen::VectorXd x; // 11 维状态向量 [xc, vx, yc, vy, zc, vz, yaw, w, r, ...]
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0;
  double radius = 0.20;
  std::vector<Eigen::Vector4d> armor_xyza_list; // 4 块拟合装甲板 [x, y, z, yaw]
  double last_nis = 0.0;
  bool diverged = false;
};

// 7. 节点间"话题"消息类型
// /armor_observation 话题（SimNode 发布 → TrackerNode 订阅），仿真时钟打戳
struct StampedObservation
{
  ArmorObservation obs;
  double sim_t = 0.0;    // 确定性仿真时钟（frame * dt），与墙钟无关
  uint64_t frame = 0;
  Eigen::Vector3d ego_acc = Eigen::Vector3d::Zero(); // gimbal 系 xy 加速度控制量，z 不用
};

// Kalman − gimbal 系真值。放在 types 以便 TrackerOutput 携带，避免 metrics↔types 循环。
struct MetricsResult
{
  double err_x = 0.0;   // m
  double err_y = 0.0;
  double err_z = 0.0;
  double err_vx = 0.0;  // m/s
  double err_vy = 0.0;
  double err_vz = 0.0;
  double err_yaw = 0.0; // rad, (-pi, pi]
  double err_w = 0.0;   // rad/s
  double err_r = 0.0;   // m

  double pos_err_norm = 0.0;
  double vel_err_norm = 0.0;
  double armor_err_norm = 0.0;

  double flight_time = 0.0;
  double aim_lead_bias = 0.0;
  double nis = 0.0;

  uint64_t samples = 0;
  double rmse_pos = 0.0;
  double rmse_vel = 0.0;
  double mean_pos = 0.0;
  double mean_vel = 0.0;
  double max_pos = 0.0;
  double max_vel = 0.0;
};

// /target_estimate 话题（TrackerNode 发布 → 可视化读取）
struct TrackerOutput
{
  TargetEstimate est;
  double sim_t = 0.0;
  MetricsResult metrics;
};

// 8. 仿真系统参数配置
struct SimulationConfig
{
  // 自车与偏置云台
  double chassis_w = 10.0;
  double chassis_vx = 0.0;
  double chassis_vy = 0.0;
  Eigen::Vector3d gimbal_offset{0.12, 0.0, 0.0};

  // 敌车目标
  Eigen::Vector3d target_init_pos{3.5, 0.0, 0.0};
  Eigen::Vector3d target_vel{0.0, 0.0, 0.0};
  double target_radius = 0.20;
  double target_w = 4.0;
  int armor_num = 4;

  // 测量噪声
  bool enable_noise = true;
  double noise_pos_std = 0.005;
  double noise_yaw_std = 0.01;
  double noise_acc_std = 1.0;  // 仿真 IMU xy 高斯噪声标准差 (m/s^2)；应对齐 ekf.m_ax / m_ay 开方

  // 仿真运行
  int fps = 50;
  double duration = 0.0; // 0: 无限循环
  double bullet_speed = 25.0; // m/s
};

}  // namespace common

#endif  // COMMON__TYPES_HPP

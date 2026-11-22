#ifndef COMMON__TYPES_HPP
#define COMMON__TYPES_HPP

#include <Eigen/Dense>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace common
{

// =============================================================================
// 坐标系
//   odom   场地惯性系（中间件）。运动学在此积分；仅 TF 使用这些坐标。
//   gimbal 云台中心、轴向与 odom 平行（不转）。观测、EKF、plot、误差都在此。
//   camera 与 gimbal 同原点，轴向随 look-at yaw。只给光轴 TF。
//   robot  敌车体。装甲板绑在此系；robot 本身在 odom 中转动。
//
// GroundTruth 的 chassis / gimbal.pos / camera / target 位姿都是 odom。
// 日常使用请走 frames.hpp 的 express_*_in_gimbal()，不要直接拿 odom 坐标滤波。
// xyz_in_gimbal / EKF / kal_* / err_* 在方向不变的 gimbal。
// xyz_in_world 保留为装甲在 odom 的位置，滤波和 plot 不用。
// =============================================================================

struct ChassisState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0;
};

struct GimbalState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
};

struct CameraState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Matrix3d R_camera2odom = Eigen::Matrix3d::Identity();
};

struct TargetState
{
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0;
  double radius = 0.20;
  double l = 0.0;
  double h = 0.0;
  int armor_num = 4;
  std::vector<Eigen::Vector3d> armors_pos;
  std::vector<double> armors_yaw;
};

struct GroundTruth
{
  double t = 0.0;
  ChassisState chassis;
  GimbalState gimbal;
  CameraState camera;
  TargetState target;
  int visible_armor_id = 0;
};

struct ArmorObservation
{
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();
  double yaw_raw = 0.0;
  int armor_id = 0;
  double confidence = 0.98;
  std::string name = "one";
  std::string color = "blue";
  std::string type = "small";
  double width = 0.135;
  double height = 0.056;
};

struct TargetEstimate
{
  Eigen::VectorXd x;
  Eigen::Vector3d center = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0;
  double radius = 0.20;
  double l = 0.0;
  double h = 0.0;
  bool jumped = false;
  int armor_num = 4;
  std::vector<Eigen::Vector4d> armor_xyza_list;
  double last_nis = 0.0;
  bool diverged = false;
};

struct StampedObservation
{
  ArmorObservation obs;
  double sim_t = 0.0;
  uint64_t frame = 0;
  Eigen::Vector3d ego_acc = Eigen::Vector3d::Zero();
};

struct MetricsResult
{
  double err_x = 0.0;
  double err_y = 0.0;
  double err_z = 0.0;
  double err_vx = 0.0;
  double err_vy = 0.0;
  double err_vz = 0.0;
  double err_yaw = 0.0;
  double err_w = 0.0;
  double err_r = 0.0;
  double err_l = 0.0;
  double err_h = 0.0;

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

struct GimbalCommand
{
  bool valid = false;
  double target_yaw = 0.0;
  double target_pitch = 0.0;
  double yaw = 0.0;
  double pitch = 0.0;
  double yaw_vel = 0.0;
  double yaw_acc = 0.0;
  double pitch_vel = 0.0;
  double pitch_acc = 0.0;
  double fly_time = 0.0;
  Eigen::Vector3d aim_xyz = Eigen::Vector3d::Zero();
};

struct AimerParams
{
  double yaw_offset = 0.0;
  double pitch_offset = 0.0;
  double coming_angle = 55.0 * M_PI / 180.0;
  double leaving_angle = 20.0 * M_PI / 180.0;
  double delay_time = 0.0;
};

struct PlannerParams
{
  double max_yaw_acc = 50.0;
  Eigen::Vector2d Q_yaw{9.0e6, 0.0};
  double R_yaw = 1.0;
  double max_pitch_acc = 100.0;
  Eigen::Vector2d Q_pitch{9.0e6, 0.0};
  double R_pitch = 1.0;
};

struct TrackerOutput
{
  TargetEstimate est;
  double sim_t = 0.0;
  MetricsResult metrics;
};

struct SimulationConfig
{
  double chassis_w = 10.0;
  double chassis_vx = 0.0;
  double chassis_vy = 0.0;
  Eigen::Vector3d gimbal_offset{0.12, 0.0, 0.0};
  double chassis_x_min = -1.5;
  double chassis_x_max = 1.5;
  double chassis_y_min = -1.5;
  double chassis_y_max = 1.5;

  Eigen::Vector3d target_init_pos{3.5, 0.0, 0.0};
  Eigen::Vector3d target_vel{0.0, 0.0, 0.0};
  double target_x_min = 2.0;
  double target_x_max = 5.0;
  double target_y_min = -1.5;
  double target_y_max = 1.5;
  double target_radius = 0.20;
  double target_l = 0.05;
  double target_h = 0.05;
  double target_w = 4.0;
  int armor_num = 4;

  bool enable_noise = true;
  double noise_pos_std = 0.005;
  double noise_yaw_std = 0.01;
  double noise_acc_std = 1.0;

  int fps = 50;
  double duration = 0.0;
  double bullet_speed = 25.0;

  AimerParams aimer;
  PlannerParams planner;
};

}  // namespace common

#endif  // COMMON__TYPES_HPP

#ifndef COMMON__TYPES_HPP
#define COMMON__TYPES_HPP

#include <Eigen/Dense>
#include <cstdint>
#include <string>
#include <vector>

namespace common
{

// 1. 本车底盘状态
struct ChassisState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double w = 0.0; // 自转角速度 (rad/s)
};

// 2. 本车公转云台状态
// 注：云台相对底盘的偏置由配置给出，实际杆臂矢量 = gimbal.pos - chassis.pos
struct GimbalState
{
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double pitch = 0.0;
  Eigen::Matrix3d R_gimbal2world = Eigen::Matrix3d::Identity();
};

// 3. 敌车目标真实状态
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

// 4. 仿真环境综合真实物理状态 (GroundTruth)
struct GroundTruth
{
  double t = 0.0;
  ChassisState chassis;
  GimbalState gimbal;
  TargetState target;

  // 当前由视觉传感器认定的最佳可见装甲板 id，
  // 位姿请按 id 索引 target.armors_pos / target.armors_yaw（不再冗余存储）
  int visible_armor_id = 0;
};

// 5. 传感器产生的装甲板视觉测量 (Observation)
struct ArmorObservation
{
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();
  Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
  Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero(); // yaw, pitch, distance
  double yaw_raw = 0.0;
  double confidence = 0.98;
  std::string name = "one";
  std::string color = "blue";
  std::string type = "small";
  double width = 0.135;
  double height = 0.056;
};

// 6. 算法给出的目标状态估计 (TargetEstimate)
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
};

// /target_estimate 话题（TrackerNode 发布 → 可视化读取）
struct TrackerOutput
{
  TargetEstimate est;
  double sim_t = 0.0;
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

  // 仿真运行
  int fps = 50;
  double duration = 0.0; // 0: 无限循环
  double bullet_speed = 25.0; // m/s
};

}  // namespace common

#endif  // COMMON__TYPES_HPP

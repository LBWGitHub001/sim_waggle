#include "camera_sensor.hpp"

#include <cmath>

namespace sim
{
namespace
{
Eigen::Vector3d xyz2ypd(const Eigen::Vector3d & xyz)
{
  double d = xyz.norm();
  double yaw = std::atan2(xyz.y(), xyz.x());
  double pitch = std::atan2(xyz.z(), xyz.head<2>().norm());
  return {yaw, pitch, d};
}
}  // namespace

CameraSensor::CameraSensor(const common::SimulationConfig & cfg)
: cfg_(cfg),
  dist_pos_(0.0, cfg.noise_pos_std),
  dist_yaw_(0.0, cfg.noise_yaw_std)
{
}

common::ArmorObservation CameraSensor::observe(const common::GroundTruth & gt)
{
  common::ArmorObservation obs;
  obs.name = "one";
  obs.color = "blue";
  obs.type = "small";
  obs.confidence = 0.98;
  obs.width = 0.135;
  obs.height = 0.056;

  // 当前可见装甲板的真值位姿
  const auto & armor_pos = gt.target.armors_pos[gt.visible_armor_id];
  const double armor_yaw = gt.target.armors_yaw[gt.visible_armor_id];

  // 1. 云台指向装甲板的相对矢量（世界系）
  const Eigen::Vector3d r_g2a = armor_pos - gt.gimbal.pos;

  // 2. 云台系坐标（相机原始测量）
  obs.xyz_in_gimbal = gt.gimbal.R_gimbal2world.transpose() * r_g2a;

  // 3. "伪世界系"坐标：等价于真实自瞄管线 R_gimbal2world * xyz_in_gimbal 的输出，
  //    即只做了旋转变换、未补偿云台自身平移 —— 这正是本仿真要复现的偏置误差来源
  obs.xyz_in_world = r_g2a;

  // 4. 观测高斯白噪声（平面假设：z 向故意不加噪，恒为 0）
  if (cfg_.enable_noise) {
    const Eigen::Vector3d noise(dist_pos_(rng_), dist_pos_(rng_), 0.0);
    obs.xyz_in_world += noise;
  }

  // 5. 球坐标系观测
  obs.ypd_in_world = xyz2ypd(obs.xyz_in_world);

  // 6. 装甲板姿态角观测
  obs.yaw_raw = armor_yaw + (cfg_.enable_noise ? dist_yaw_(rng_) : 0.0);

  return obs;
}

}  // namespace sim

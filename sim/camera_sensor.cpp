#include "camera_sensor.hpp"

#include <cmath>

#include "frames.hpp"

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

  const auto & armor_pos = gt.target.armors_pos[gt.visible_armor_id];
  const double armor_yaw_odom = gt.target.armors_yaw[gt.visible_armor_id];

  const Eigen::Vector3d r_g2a = armor_pos - gt.gimbal.pos;

  // 工作系：旋转相机系。噪声加在 gimbal，ypd / yaw 也在 gimbal。
  obs.xyz_in_gimbal = gt.gimbal.R_gimbal2world.transpose() * r_g2a;
  obs.xyz_in_world = armor_pos;  // odom 中间件，滤波不用
  obs.yaw_raw = common::yaw_odom_to_gimbal(armor_yaw_odom, gt.gimbal);

  if (cfg_.enable_noise) {
    const Eigen::Vector3d noise(dist_pos_(rng_), dist_pos_(rng_), 0.0);
    obs.xyz_in_gimbal += noise;
    obs.yaw_raw += dist_yaw_(rng_);
  }

  obs.ypd_in_world = xyz2ypd(obs.xyz_in_gimbal);
  return obs;
}

}  // namespace sim

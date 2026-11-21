#ifndef SIM__CAMERA_SENSOR_HPP
#define SIM__CAMERA_SENSOR_HPP

#include <random>
#include "types.hpp"

namespace sim
{

/// 车载相机视觉感知传感器模拟器
/// 取真值中当前可见装甲板，生成带高斯噪声的视觉观测 ArmorObservation。
/// 工作系是 gimbal：xyz_in_gimbal / ypd / yaw_raw。xyz_in_world 是装甲 odom 坐标（中间件）。
class CameraSensor
{
public:
  explicit CameraSensor(const common::SimulationConfig & cfg = common::SimulationConfig{});

  common::ArmorObservation observe(const common::GroundTruth & gt);

private:
  common::SimulationConfig cfg_;
  std::default_random_engine rng_{42};
  std::normal_distribution<double> dist_pos_;
  std::normal_distribution<double> dist_yaw_;
};

}  // namespace sim

#endif  // SIM__CAMERA_SENSOR_HPP

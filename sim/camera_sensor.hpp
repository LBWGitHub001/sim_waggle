#ifndef SIM__CAMERA_SENSOR_HPP
#define SIM__CAMERA_SENSOR_HPP

#include <random>
#include "types.hpp"

namespace sim
{

/// 车载相机视觉感知传感器模拟器
/// 取真值中当前可见装甲板，生成带高斯噪声的视觉观测 ArmorObservation。
/// 注意：xyz_in_world 是"未补偿云台平移"的伪世界系坐标（与真实自瞄管线一致），
/// 这是本仿真复现偏置伪速度的关键语义。
class CameraSensor
{
public:
  explicit CameraSensor(const common::SimulationConfig & cfg = common::SimulationConfig{});

  // 根据当前物理真值生成视觉观测 ArmorObservation
  common::ArmorObservation observe(const common::GroundTruth & gt);

private:
  common::SimulationConfig cfg_;
  std::default_random_engine rng_{42};
  std::normal_distribution<double> dist_pos_;
  std::normal_distribution<double> dist_yaw_;
};

}  // namespace sim

#endif  // SIM__CAMERA_SENSOR_HPP

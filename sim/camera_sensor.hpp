#ifndef SIM__CAMERA_SENSOR_HPP
#define SIM__CAMERA_SENSOR_HPP

#include <random>
#include "types.hpp"

namespace sim
{

/// 车载相机视觉感知传感器模拟器
/// 取真值中当前可见装甲板，生成带高斯噪声的视觉观测 ArmorObservation。
/// xyz_in_world 在世界系（原点=云台中心，轴向固定）；xyz_in_gimbal 在相机系。
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

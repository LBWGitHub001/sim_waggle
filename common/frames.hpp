#ifndef COMMON__FRAMES_HPP
#define COMMON__FRAMES_HPP

#include <cmath>

#include "types.hpp"

namespace common
{

inline double wrap_pi(double angle)
{
  constexpr double k2Pi = 2.0 * M_PI;
  double a = std::fmod(angle + M_PI, k2Pi);
  if (a < 0.0) a += k2Pi;
  return a - M_PI;
}

// robot 系固定装甲：第 i 块在车体前方为 -x（与历史几何一致）
inline Eigen::Vector3d armor_in_robot(int i, int armor_num, double radius)
{
  const double yaw_i = static_cast<double>(i) * 2.0 * M_PI / static_cast<double>(armor_num);
  return Eigen::Vector3d(-radius * std::cos(yaw_i), -radius * std::sin(yaw_i), 0.0);
}

inline double armor_yaw_in_robot(int i, int armor_num)
{
  return static_cast<double>(i) * 2.0 * M_PI / static_cast<double>(armor_num);
}

inline Eigen::Vector3d point_odom_to_gimbal(const Eigen::Vector3d & p_odom, const GimbalState & g)
{
  return g.R_gimbal2world.transpose() * (p_odom - g.pos);
}

// 只减云台线速度再旋到 gimbal 轴，不做 ω×r
inline Eigen::Vector3d vel_odom_to_gimbal(const Eigen::Vector3d & v_odom, const GimbalState & g)
{
  return g.R_gimbal2world.transpose() * (v_odom - g.vel);
}

inline double yaw_odom_to_gimbal(double yaw_odom, const GimbalState & g)
{
  return wrap_pi(yaw_odom - g.yaw);
}

inline TargetState express_target_in_gimbal(const GroundTruth & gt)
{
  TargetState t = gt.target;
  t.center = point_odom_to_gimbal(gt.target.center, gt.gimbal);
  t.vel = vel_odom_to_gimbal(gt.target.vel, gt.gimbal);
  t.yaw = yaw_odom_to_gimbal(gt.target.yaw, gt.gimbal);
  for (size_t i = 0; i < t.armors_pos.size(); ++i) {
    t.armors_pos[i] = point_odom_to_gimbal(gt.target.armors_pos[i], gt.gimbal);
    t.armors_yaw[i] = yaw_odom_to_gimbal(gt.target.armors_yaw[i], gt.gimbal);
  }
  return t;
}

}  // namespace common

#endif  // COMMON__FRAMES_HPP

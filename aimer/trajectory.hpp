#ifndef COMMON__TRAJECTORY_HPP
#define COMMON__TRAJECTORY_HPP

namespace common
{

/// 无阻力弹道。pitch 抬头为正。
struct BulletTrajectory
{
  bool unsolvable = true;
  double fly_time = 0.0;
  double pitch = 0.0;

  // v0 弹速 m/s，d 水平距离 m，h 目标高度 m
  BulletTrajectory(double v0, double d, double h);
};

}  // namespace common

#endif

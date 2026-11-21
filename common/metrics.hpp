#ifndef COMMON__METRICS_HPP
#define COMMON__METRICS_HPP

#include "types.hpp"

namespace common
{

/// 误差 = Kalman 状态 − gimbal 系敌车真值（express_target_in_gimbal）。
/// 调用方应传入已变到 gimbal 的 GroundTruth（target 字段已是工作系），
/// 或由 TrackerNode 先 express 再比较。
class SimulationMetrics
{
public:
  SimulationMetrics() = default;

  MetricsResult update(
    const GroundTruth & gt,
    const TargetEstimate & est,
    double bullet_speed = 25.0);

  const MetricsResult & latest() const { return latest_; }

  void reset();

private:
  MetricsResult latest_;

  uint64_t samples_ = 0;
  double sum_sq_pos_ = 0.0;
  double sum_sq_vel_ = 0.0;
  double sum_pos_ = 0.0;
  double sum_vel_ = 0.0;
  double max_pos_ = 0.0;
  double max_vel_ = 0.0;
};

Eigen::Vector3d compute_aim_lead_point(const TargetEstimate & est, double flight_time);

}  // namespace common

#endif  // COMMON__METRICS_HPP

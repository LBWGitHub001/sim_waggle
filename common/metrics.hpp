#ifndef COMMON__METRICS_HPP
#define COMMON__METRICS_HPP

#include <cstdint>

#include "types.hpp"

namespace common
{

/// 误差 = Kalman 状态 − 辅助惯性系敌车真值（target_inertial）。
/// 不要拿世界系表观 target.center/vel 当敌车真值，否则杆臂鬼影会被抵消。
struct MetricsResult
{
  double err_x = 0.0;   // m   kal.center - gt.target_inertial.center
  double err_y = 0.0;
  double err_z = 0.0;
  double err_vx = 0.0;  // m/s kal.velocity - gt.target_inertial.vel
  double err_vy = 0.0;
  double err_vz = 0.0;
  double err_yaw = 0.0; // rad, (-pi, pi]
  double err_w = 0.0;   // rad/s
  double err_r = 0.0;   // m   kal.radius - gt.target.radius

  double pos_err_norm = 0.0;  // |Δcenter| (m)
  double vel_err_norm = 0.0;  // |Δvel| (m/s)
  double armor_err_norm = 0.0; // 拟合板到最近真值板均距 (m)

  double flight_time = 0.0;   // s, |kal.center| / bullet_speed
  double aim_lead_bias = 0.0; // |aim_kal - aim_true| (m)
  double nis = 0.0;

  uint64_t samples = 0;
  double rmse_pos = 0.0;
  double rmse_vel = 0.0;
  double mean_pos = 0.0;
  double mean_vel = 0.0;
  double max_pos = 0.0;
  double max_vel = 0.0;
};

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

// 由状态快照计算弹道超前瞄准落点（可视化 / 真值对照同一公式）
Eigen::Vector3d compute_aim_lead_point(const TargetEstimate & est, double flight_time);

}  // namespace common

#endif  // COMMON__METRICS_HPP
